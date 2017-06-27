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

# include "MAX17201.hpp"

#define TO_PERCENTAGE	(1./256)
#define TO_CAPACITY		(0.005/R_SENSE)			// mAh
#define TO_VOLTAGE		0.078125				// mV
#define TO_CURRENT		(1.5625/(R_SENSE*1000))	// mA
#define TO_TEMP			(1./256)				// °C
#define TO_RESISTANCE	(1./4096)				// Ω
#define TO_SECONDS		5.625					// s

MAX17201::MAX17201(I2C* i2c, PinName interruptPin):
	_i2cAddress(I2CAddress::ModelGaugeM5Address), _interruptPin(interruptPin)
{
	_i2c = i2c;
}

bool MAX17201::configure(uint8_t number_of_cells, uint16_t design_capacity, float empty_voltage,
			bool use_external_thermistor1, bool use_external_thermistor2)
{
	if (number_of_cells > 15) {
		printf("Invalid number of cells ! 15 max allowed\n");
		return false;
	}

	//check gauge presency:
	if (_i2c->write(static_cast<int>(_i2cAddress) << 1, NULL, 0) != 0){
		return false;
	}

	uint8_t temp1;
	if (use_external_thermistor1){
		temp1 = 1;
	}
	else {
		temp1 = 0;
	}

	uint8_t temp2;
	if (use_external_thermistor2){
		temp2 = 1;
	}
	else {
		temp2 = 0;
	}

	uint16_t config = (1 << 15) | 				// Fuel Gauge Temperature
					  (0 << 14) | 				// Should always be 0
					  ((temp2 & 0x1) << 13) |	// 1 if a thermistor is present on AIN2
					  ((temp1 & 0x1) << 12) |	// Thermistor 1
					  (1 << 11) |				// Use internal thermistor
					  (0 << 10) |				// we use default parameter
					  (0 << 9)  |				// we use default parameter
					  (0 << 8)  |				// we use default parameter
					  (0 << 7)  |				// we use default parameter
					  (0 << 6)  |				// we use default parameter
					  (0 << 5)  |				// we use default parameter
					  (0 << 4)  |				// Should always be 0
					  (number_of_cells & 0x0F);

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}
	i2c_set_register(RegisterAddress::PackCfg, config);

	set_empty_voltage(empty_voltage);
	set_design_capacity(design_capacity);

	restart_firmware();
	return true;
}

float MAX17201::state_of_charge()
{
	float SOC;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::RepSOC, &value);

	SOC = value*TO_PERCENTAGE;
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

/** Get the average temperature for a configurable period (default is 1.5min)
 *
 * @returns
 *      The average temperature
 */
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

	float age = value * 3.4;
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

/** Configure the empty voltage used by ModelGauge m5 algorithm
 * 					_____________________________________________________________________________________
 * Register format |_D15_|_D14_|_D13_|_D12_|_D11_|_D10_|_D9_|_D8_|_D7_|_D6_|_D5_|_D4_|_D3_|_D2_|_D1_|_D0_|
 * 				   |_____________________VEmpty_______________________|___________VRecovery______________|
 * VEmpty is 10mv per LSB
 * VRecovery is 40mv per LSB
 *
 * @param VEmpty The empty voltage to use in Volts
 */
void MAX17201::set_empty_voltage(float VEmpty)
{
	uint16_t data;
	data = ((static_cast<uint16_t>(VEmpty*100) & 0x1FF) << 7) | (static_cast<uint8_t>((VEmpty+0.5)*25) & 0x7F);

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_set_register(RegisterAddress::VEmpty, data);
}

void MAX17201::set_design_capacity(uint16_t design_capacity)
{
	uint16_t data;
	data = static_cast<uint16_t>(design_capacity/TO_CAPACITY);

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_set_register(RegisterAddress::DesignCap, data);
}

void MAX17201::restart_firmware()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t restart_cmd = 0x0001;
	i2c_set_register(RegisterAddress::Config2, restart_cmd);
	wait_ms(10);
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

void MAX17201::handle_alert()
{
	//TODO : maybe use this function as a callback when an interrupt occurs on the interrupt pin ?
}

int MAX17201::i2c_set_register(RegisterAddress address, uint16_t value)
{
	static char data[3];
	data[0] = static_cast<char>(address);
	data[1] = value & 0xFF;
	data[2] = (value & 0xFF00) >> 8;

    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 3, false) != 0) {
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

    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 3, false) != 0) {
        return -1;
    }
    return 0;
}

int MAX17201::i2c_read_register(RegisterAddress address, uint16_t *value)
{
    static char data[2];
    data[0] = static_cast<char>(address);

    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data , 2, false) != 0) {
        return -2;
    }

    *value = (data[1] << 8) | data[0];
    return 0;
}

int MAX17201::i2c_read_register(RegisterAddress address, int16_t *value)
{
    static char data[2];
    data[0] = static_cast<char>(address);

    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data , 2, false) != 0) {
        return -2;
    }

    *value = (data[1] << 8) | data[0];
    return 0;
}
