/*
 * Copyright (c) 2016, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "max17201.h"

namespace sixtron {

namespace {

#define TO_PERCENTAGE     (1./256)
#define TO_CAPACITY       (0.005/R_SENSE)          // mAh
#define TO_VOLTAGE        0.078125                 // mV
#define TO_CURRENT        (1.5625/(R_SENSE*1000))  // mA
#define TO_TEMP           (1./256)                 // °C
#define TO_RESISTANCE     (1./4096)                // Ω
#define TO_SECONDS        5.625                    // s

}



MAX17201::MAX17201(I2C* i2c, PinName interruptPin):
    _i2cAddress(I2CAddress::ModelGaugeM5Address), _interruptPin(interruptPin)
{
    _i2c = i2c;
    _interruptPin.mode(OpenDrain);
}

MAX17201::MAX17201(I2C* i2c):
    _i2cAddress(I2CAddress::ModelGaugeM5Address), _interruptPin(PinName(NC))
{
    _i2c = i2c;
}

bool MAX17201::configure(uint8_t number_of_cells, uint16_t design_capacity, float empty_voltage,
            bool use_external_thermistor1, bool use_external_thermistor2, bool enable_alert)
{
    if (number_of_cells > 15) {
        printf("Invalid number of cells ! 15 max allowed\n");
        return false;
    }

    //check gauge presency:
    if (_i2c->write(static_cast<int>(_i2cAddress), NULL, 0) != 0){
        return false;
    }

    reset();

    if (_i2cAddress != I2CAddress::ShadowRAMaddress){
        _i2cAddress = I2CAddress::ShadowRAMaddress;
    }

    /* ==== Non-volatile restore configuration ==== */
    uint16_t config;
    /* nNVCfg0 configuration: */
    config = (0 << 15)  | // default cell model
             (0 << 14)  | // default cell model
             (0 << 13)  | // should always be 0
             (0 << 12)  | // should always be 0
             (0 << 11)  | // disable config and config2 restore
             (0 << 10)  | // disable filter restore
             (0 << 9)   | // disable relax config restore
             (1 << 8)   | // enable learned configuration restore
             (0 << 7)   | // disable IChgTerm restore
             (0 << 6)   | // disable CGain and COff restore
             (1 << 5)   | // enable VEmpty restore
             (1 << 4)   | // enable DesignCap restore
             (0 << 3)   | // disable MisCfg restore
             (0 << 2)   | // disable age forecasting
             (0 << 1)   | // disable HibCfg restore
             (0 << 0);    // disable SBS

    i2c_set_register(RegisterAddress::nNVCfg0, config); // nNVCfg0

    /* nNVCfg1 configuration: */
    config = (0 << 15)  | // enable TGain and Toff restore
             (0 << 14)  | // disable CGTempCo restore
             (0 << 13)  | // disable FullSOCThr restore
             (0 << 12)  | // disable RFast and VShdnCfg restore
             (0 << 11)  | // disable overcurrent comparators
             (0 << 10)  | // should always be 0
             (0 << 9)   | // should always be 0
             (0 << 8)   | // should always be 0
             (0 << 7)   | // should always be 0
             (0 << 6)   | // should always be 0
             (0 << 5)   | // should always be 0
             (0 << 4)   | // use default Time to Full configuration
             (0 << 3)   | // disable alert threshold restore
             (1 << 2)   | // enable curve correction on thermistor
             (1 << 1)   | // enable Converge to Empty functionalities
             (0 << 0);    // should always be 0

    i2c_set_register(RegisterAddress::nNVCfg1, config); // nNVCfg1

    /* nNVCfg2 configuration: */
    config = (1 << 15)  | // enable TimerH restore
             (1 << 14)  | // enable MixSOC and VFSOC backup
             (1 << 13)  | // enable MaxMinTemp restore
             (1 << 12)  | // enable MaxMinVolt restore
             (1 << 11)  | // enable MaxMinCurr restore
             (1 << 10)  | // enable average VCell and Temp restore
             (1 << 9)   | // enable fullCap and FullCapRep restore
             (1 << 8)   | // enable average empty current learning restore
             (0 << 7)   | // disable metal current sensing
             (0 << 6)   |
             (0 << 5)   |
             (0 << 4)   |
             (1 << 3)   |
             (0 << 2)   |
             (0 << 1)   |
             (1 << 0); // bit 0 to 6 defines de number of cycles between each backup operations \
                          limited to 202 backups \
                          1 LSB = 0.5 cycles \
                          We configure to perform backup every 5 cycles

    i2c_set_register(RegisterAddress::nNVCfg2, config); // nNVCfg2

    /* ==== nConfig register configuration ==== */

    config = (0 << 15)  | // should always be 0
             (0 << 14)  | // autoclear SOC alerts
             (0 << 13)  | // autoclear temperature alerts
             (0 << 12)  | // autoclear Voltage alerts
             (0 << 11)  | // disable AIN1 shutdown
             (0 << 10)  | // should always be 0
             (1 << 9)   | // enable temperature measurement
             (0 << 8)   | // use temperature measurement as decribed by PackCfg register
             (0 << 7)   | // do not shutdown
             (0 << 6)   | // disable communication shutdown capabilities
             (0 << 5)   |
             (1 << 4)   | // should always be 1
             (0 << 3)   | // disable fast thermistor switch bias control
             (0 << 2)   | // disable general alerts (Current, Voltage, SOC)
             (0 << 1)   | // disable 1% SOC change alert
             (0 << 0);    // disable temperature alerts

    i2c_set_register(RegisterAddress::nConfig, config); // nConfig

    /* ==== Thermistor configuration ==== */

    uint8_t temp1 = 0;
    uint8_t fgt = 0;
    if (use_external_thermistor1){
        temp1 = 1;
        fgt = 1;
    }

    uint8_t temp2 = 0;
    if (use_external_thermistor2){
        temp2 = 1;
        fgt = 0;
    }

    uint8_t internal_temp = 0;
    if (!use_external_thermistor1 && !use_external_thermistor2) {
        internal_temp = 1;
    }

    config = (fgt << 15)           |    // Temperature source selection
             (0 << 14)             |    // Should always be 0
             (temp2 << 13)         |    // 1 if a thermistor is present on AIN2
             (temp1 << 12)         |    // 1 if a thermistor is present on AIN1
             (internal_temp << 11) |    // Use internal thermistor
             (1 << 10)             |    // we use default parameter
             (0 << 9)              |    // we use default parameter
             (0 << 8)              |    // we use default parameter
             (0 << 7)              |    // we use default parameter
             (0 << 6)              |    // we use default parameter
             (0 << 5)              |    // we use default parameter
             (0 << 4)              |    // Should always be 0
             (number_of_cells & 0x0F);

    i2c_set_register(RegisterAddress::nPackCfg, config); // nPackCfg

    /* Configure thermistor as Murata by default: */
    if (temp1 || temp2){
        configure_thermistor(0xEE56, 0x1DA4);
    }

    set_empty_voltage(empty_voltage);
    set_design_capacity(design_capacity);

    restart_firmware();
    
    _i2cAddress = I2CAddress::ModelGaugeM5Address;

    config = (fgt << 15)           |    // Temperature source selection
             (0 << 14)             |    // Should always be 0
             (temp2 << 13)         |    // 1 if a thermistor is present on AIN2
             (temp1 << 12)         |    // 1 if a thermistor is present on AIN1
             (internal_temp << 11) |    // Use internal thermistor
             (1 << 10)             |    // we use default parameter
             (0 << 9)              |    // we use default parameter
             (0 << 8)              |    // we use default parameter
             (0 << 7)              |    // we use default parameter
             (0 << 6)              |    // we use default parameter
             (0 << 5)              |    // we use default parameter
             (0 << 4)              |    // Should always be 0
             (number_of_cells & 0x0F);

    i2c_set_register(RegisterAddress::PackCfg, config); // PackCfg

    wait_ms(100); // let time to software to compute new values


    if(enable_alert)
    {
       	// here, configure the limits values of alerts...

    	// alert max17201 enable
    	enable_alerts();
    	enable_temperature_alerts();

    	_interruptPin.enable_irq(); // interruption enable

    	wait_ms(100); // let time to software to compute new values
    }


    return true;
}

uint16_t MAX17201::status()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }
    uint16_t status;
    i2c_read_register(RegisterAddress::Status, &status);

    return status;
}

float MAX17201::state_of_charge()
{
    float SOC;
    uint16_t value;

    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    i2c_read_register(RegisterAddress::RepSOC, &value);

    SOC = value * TO_PERCENTAGE;
    SOC = SOC > 100 ? 100 : SOC;
    SOC = SOC < 0 ? 0 : SOC;
    return SOC;
}

double MAX17201::current()
{
    double current;
    int16_t value;

    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    i2c_read_register(RegisterAddress::Current, &value);

    current = value*TO_CURRENT;
    return current;
}

double MAX17201::average_current()
{
    double current;
    int16_t value;

    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    i2c_read_register(RegisterAddress::AvgCurrent, &value);

    current = value*TO_CURRENT;
    return current;
}

float MAX17201::max_current()
{
    float current;
    uint16_t value;

    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    i2c_read_register(RegisterAddress::MaxMinCurr, &value);

    current = ((value & 0xFF00) >> 8) * 0.40/R_SENSE;
    return current;
}

float MAX17201::min_current()
{
    float current;
    uint16_t value;

    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    i2c_read_register(RegisterAddress::MaxMinCurr, &value);

    current = (value & 0x00FF) * 0.40/R_SENSE;
    return current;
}

double MAX17201::cell_voltage()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::VCell, &value);

    double VCell = value*TO_VOLTAGE;
    return VCell;
}

double MAX17201::average_cell_voltage()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::AvgVCell, &value);

    double VCell = value*TO_VOLTAGE;
    return VCell;
}

float MAX17201::max_cell_voltage()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::MaxMinVolt, &value);

    float max_voltage = ((value & 0xFF00) >> 8) * 0.20;
    return max_voltage;
}

float MAX17201::min_cell_voltage()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::MaxMinVolt, &value);

    float min_voltage = (value & 0x00FF) * 0.20;
    return min_voltage;
}

float MAX17201::reported_capacity()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::Config, &value);
    i2c_read_register(RegisterAddress::IAlrtTh, &value);
    i2c_read_register(RegisterAddress::RepCap, &value);

    float cap = value*TO_CAPACITY;
    return cap;
}

float MAX17201::full_capacity()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::FullCapRep, &value);

    float cap = value*TO_CAPACITY;
    return cap;
}

float MAX17201::design_capacity()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::DesignCap, &value);

    float cap = value*TO_CAPACITY;
    return cap;
}

float MAX17201::time_to_empty()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::TTE, &value);

    float time_to_empty = value*TO_SECONDS;
    return time_to_empty;
}

float MAX17201::time_to_full()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::TTF, &value);

    float time_to_full = value*TO_SECONDS;
    return time_to_full;
}

float MAX17201::temperature()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    int16_t value;
    i2c_read_register(RegisterAddress::Temp, &value);

    float temperature = value*TO_TEMP;
    return temperature;
}

float MAX17201::average_temperature()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    int16_t value;
    i2c_read_register(RegisterAddress::AvgTA, &value);

    float avg_temperature = value*TO_TEMP;
    return avg_temperature;
}

int8_t MAX17201::max_temperature()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::MaxMinTemp, &value);

    int8_t max_temperature = (value & 0xFF00) >> 8;
    return max_temperature;
}

int8_t MAX17201::min_temperature()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::MaxMinTemp, &value);

    int8_t min_temperature = (value & 0x00FF);
    return min_temperature;
}

float MAX17201::age()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::TimerH, &value);

    float age = value * 3.2;
    return age;
}

float MAX17201::cycle_count()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t value;
    i2c_read_register(RegisterAddress::Cycles, &value);

    float cycles = value * (16/100);
    return cycles;
}

void MAX17201::set_temperature_alerts(int8_t max_temperature_threshold, int8_t min_temperature_threshold)
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t data = ((max_temperature_threshold & 0xFF) << 8) | (min_temperature_threshold & 0xFF); // set threshold
    i2c_set_register(RegisterAddress::TAlrtTh, data);
}

void MAX17201::set_current_alerts(float max_current_threshold, float min_current_threshold)
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    int8_t max_thr = (max_current_threshold*1000*R_SENSE)/400;
    int8_t min_thr = (min_current_threshold*1000*R_SENSE)/400;

    uint16_t data = ((max_thr & 0xFF) << 8) | (min_thr & 0xFF); // set threshold
    i2c_set_register(RegisterAddress::IAlrtTh, data);
}

void MAX17201::set_voltage_alerts(float max_voltage_threshold, float min_voltage_threshold)
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint8_t max_thr = (max_voltage_threshold*1000)/20;
    uint8_t min_thr = (min_voltage_threshold*1000)/20;

    uint16_t data = ((max_thr & 0xFF) << 8) | (min_thr & 0xFF); // set threshold
    i2c_set_register(RegisterAddress::VAlrtTh, data);
}

void MAX17201::set_state_of_charge_alerts(uint8_t max_soc_threshold, uint8_t min_soc_threshold)
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t data = (max_soc_threshold << 8) | min_soc_threshold; // set threshold
    i2c_set_register(RegisterAddress::SAlrtTh, data);
}

void MAX17201::enable_alerts()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }
    uint16_t config;
    i2c_read_register(RegisterAddress::Config, &config); // nConfig
    config |= (1 << 2); // Enable alerts
    i2c_set_register(RegisterAddress::Config, config); // write back config
}

void MAX17201::enable_temperature_alerts()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }
    uint16_t config;
    i2c_read_register(RegisterAddress::Config2, &config); // nConfig
    config |= (1 << 6); // Enable temperature alerts
    i2c_set_register(RegisterAddress::Config2, config); // write back config
}

void MAX17201::disable_alerts()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }
    uint16_t config;
    i2c_read_register(RegisterAddress::Config, &config); // nConfig
    config = config &(0xFFFB); // Disable alerts
    i2c_set_register(RegisterAddress::Config, config); // write back config
}

void MAX17201::disable_temperature_alerts()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }
    uint16_t config;
    i2c_read_register(RegisterAddress::Config2, &config); // nConfig
    config = config &(0xFFBF); // Disable temperature alerts
    i2c_set_register(RegisterAddress::Config2, config); // write back config
}

void MAX17201::configure_thermistor(uint16_t gain, uint16_t offset)
{
    if (_i2cAddress != I2CAddress::ShadowRAMaddress){
        _i2cAddress = I2CAddress::ShadowRAMaddress;
    }

    i2c_set_register(RegisterAddress::nTGain, gain);
    i2c_set_register(RegisterAddress::nTOff, offset);
}

void MAX17201::set_empty_voltage(float VEmpty)
{
    uint16_t data;
    data = ((static_cast<uint16_t>(VEmpty*100) & 0x1FF) << 7) | (static_cast<uint8_t>((VEmpty+0.5)*25) & 0x7F);

    if (_i2cAddress != I2CAddress::ShadowRAMaddress){
        _i2cAddress = I2CAddress::ShadowRAMaddress;
    }

    i2c_set_register(RegisterAddress::nVEmpty, data);
}

void MAX17201::set_design_capacity(uint16_t design_capacity)
{
    uint16_t data;
    data = static_cast<uint16_t>(design_capacity/TO_CAPACITY);

    if (_i2cAddress != I2CAddress::ShadowRAMaddress){
        _i2cAddress = I2CAddress::ShadowRAMaddress;
    }

    i2c_set_register(RegisterAddress::nDesignCap, data); // nDesignCap
    i2c_set_register(RegisterAddress::nFullCapRep, data); // nFullCapRep

    data = static_cast<uint16_t>((design_capacity*1.1)/TO_CAPACITY);
    i2c_set_register(RegisterAddress::nFullCapNom, data); // nFullCapNom
}

void MAX17201::restart_firmware()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t restart_cmd = 0x0001;
    i2c_set_register(RegisterAddress::Config2, restart_cmd);
    uint16_t reg = 1;
    do {
        uint8_t ret = i2c_read_register(RegisterAddress::Config2, &reg);
        if (ret !=0) {
            printf("i2c error\n");
        }
        reg = reg & restart_cmd;
        wait_ms(5);
    } while (reg == restart_cmd);
}

void MAX17201::reset()
{
    if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
        _i2cAddress = I2CAddress::ModelGaugeM5Address;
    }

    uint16_t reset_cmd = 0x00F;
    i2c_set_register(RegisterAddress::CmdRegister, reset_cmd); // Hardware power on reset
    wait_ms(10);
}


uint8_t MAX17201::remaining_writes()
{
    uint16_t command = 0xE2FA; // Ask for remainings update
    i2c_set_register(RegisterAddress::CmdRegister, command);
    wait_ms(5); // tRECALL

    _i2cAddress = I2CAddress::ShadowRAMaddress;
    uint16_t data = 0;
    i2c_read_register((RegisterAddress) 0xED, &data);
    _i2cAddress = I2CAddress::ModelGaugeM5Address;
    /* Compute the remainings updates
     * see the datasheet for more informations about the steps to be done
     */
    uint8_t remaining_writes = (data & 0xFF00 >> 8) | (data & 0xFF);
    int i = 0;
    int mask = 0x01;
    for (i = 0; i < 7; i++) {
        if (mask ^ remaining_writes == 0) {
            break;
        }
        else {
            mask = (mask << 1) + 1;
        }
    }
    return (7 - i);
}

/*****
 *
 *
 *
 *****/

void MAX17201::handle_alert()
{
    //TODO : maybe use this function as a callback when an interrupt occurs on the interrupt pin ?


}


void MAX17201::cleardSOCi_bit()
{
	uint16_t temp = 0;
	i2c_read_register(MAX17201::RegisterAddress::Status, &temp); // Status
	temp &= 0xff7f; // clear dSOCi bit
	i2c_set_register(MAX17201::RegisterAddress::Status, temp); // write back Status
}

/*****
 *
 *
 *
 *****/



int MAX17201::i2c_set_register(RegisterAddress address, uint16_t value)
{
    static char data[3];
    data[0] = static_cast<char>(address);
    data[1] = value & 0xFF;
    data[2] = (value & 0xFF00) >> 8;

    if (_i2c->write(static_cast<int>(_i2cAddress), data, 3, false) != 0) {
        return -1;
    }
    return 0;
}

int MAX17201::i2c_set_register(RegisterAddress address, int16_t value)
{
    static char data[3];
    data[0] = static_cast<char>(address);
    data[1] = value & 0xFF;
    data[2] = (value & 0xFF00) >> 8;

    if (_i2c->write(static_cast<int>(_i2cAddress), data, 3, false) != 0) {
        return -1;
    }
    return 0;
}

int MAX17201::i2c_read_register(RegisterAddress address, uint16_t *value)
{
    static char data[2];
    data[0] = static_cast<char>(address);

    if (_i2c->write(static_cast<int>(_i2cAddress), data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress), data , 2, false) != 0) {
        return -2;
    }

    *value = (data[1] << 8) | data[0];
    return 0;
}

int MAX17201::i2c_read_register(RegisterAddress address, int16_t *value)
{
    static char data[2];
    data[0] = static_cast<char>(address);

    if (_i2c->write(static_cast<int>(_i2cAddress), data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress), data , 2, false) != 0) {
        return -2;
    }

    *value = (data[1] << 8) | data[0];
    return 0;
}

} // namepsace sixtron
