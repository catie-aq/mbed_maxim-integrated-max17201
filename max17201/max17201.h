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
#ifndef MAX17201_H
#define MAX17201_H

#include "mbed.h"

namespace sixtron {

#define R_SENSE            0.020 //Value of the sense resistor

/*!
 *  \class MAX17201
 *  MAX17201 gauge driver
 */
class MAX17201
{
public:
    /* I2C addresses */
    enum class I2CAddress : char {
        ModelGaugeM5Address            = 0x6C,
        ShadowRAMaddress               = 0x16,
        SBSDataAddress                 = 0x16
    };

    /* Represents the different alert flags for the MAX17048 */
    enum class StatusFlags : uint16_t { // short ?
        ALERT_POR             = (1 << 1),  /*!< Power On Reset Indicator */
        ALERT_CL              = (1 << 2),  /*!< Minimum Current Alert Threshold Exceeded */
        BATTERY_PRESENT       = (1 << 3),  /*!< Battery presence indicator */
        ALERT_CH              = (1 << 6),  /*!< Maximum Current Alert Threshold Exceeded */
        ALERT_dSOCI           = (1 << 7),  /*!< 1% SOC change alert */
        ALERT_VL              = (1 << 8),  /*!< Minimum Voltage Alert Threshold Exceeded */
        ALERT_TL              = (1 << 9),  /*!< Minimum Temperature Alert Threshold Exceeded */
        ALERT_SL              = (1 << 10), /*!< Minimum SOC Alert Threshold Exceeded */
        ALERT_BI              = (1 << 11), /*!< Battery Insertion */
        ALERT_VH              = (1 << 12), /*!< Maximum Voltage Alert Threshold Exceeded */
        ALERT_TH              = (1 << 13), /*!< Maximum Temperature Alert Threshold Exceeded */
        ALERT_SH              = (1 << 14), /*!< Maximum SOC Alert Threshold Exceeded */
        ALERT_BR              = (1 << 15)  /*!< Battery Removal */
    };

    enum class RegisterAddress : char {
        /* ModelGauge m5 Registers */
        Status            = 0x00,
        VAlrtTh           = 0x01,
        TAlrtTh           = 0x02,
        SAlrtTh           = 0x03,
        AtRate            = 0x04,
        RepCap            = 0x05,
        RepSOC            = 0x06,
        Age               = 0x07,
        Temp              = 0x08,
        VCell             = 0x09,
        Current           = 0x0A,
        AvgCurrent        = 0x0B,
        QResidual         = 0x0C,
        MixSOC            = 0x0D,
        AvSOC             = 0x0E,
        MixCap            = 0x0F,

        FullCap           = 0x10,
        TTE               = 0x11,
        QRTable00         = 0x12,
        FullSocThr        = 0x13,
        RCell             = 0x14,
        RFast             = 0x15,
        AvgTA             = 0x16,
        Cycles            = 0x17,
        DesignCap         = 0x18,
        AvgVCell          = 0x19,
        MaxMinTemp        = 0x1A,
        MaxMinVolt        = 0x1B,
        MaxMinCurr        = 0x1C,
        Config            = 0x1D,
        IChgTerm          = 0x1E,
        AvCap             = 0x1F,

        TTF               = 0x20,
        DevName           = 0x21,
        QRTable10         = 0x22,
        FullCapNom        = 0x23,
        AIN0              = 0x27,
        LearnCfg          = 0x28,
        FilterCfg         = 0x29,
        RelaxCfg          = 0x2A,
        MiscCfg           = 0x2B,
        TGain             = 0x2C,
        Toff              = 0x2D,
        CGain             = 0x2E,
        COff              = 0x2F,

        QRTable20         = 0x32,
        FullCapRep        = 0x35,
        IAvgEmpty         = 0x36,
        RComp0            = 0x38,
        TempCo            = 0x39,
        VEmpty            = 0x3A,
        Fstat             = 0x3D,
        Timer             = 0x3E,
        ShdnTimer         = 0x3F,

        QRTable30         = 0x42,
        dQAcc             = 0x45,
        dPAcc             = 0x46,
        VFRemCap          = 0x4A,
        QH                = 0x4D,

        Status2            = 0xB0,
        IAlrtTh            = 0xB4,
        VShdnCfg           = 0xB8,
        AgeForecast        = 0xB9,
        HibCfg             = 0xBA,
        Config2            = 0xBB,
        VRipple            = 0xBC,
        PackCfg            = 0xBD,
        TimerH             = 0xBE,

        AvgCell4           = 0xD1,
        AvgCell3           = 0xD2,
        AvgCell2           = 0xD3,
        AvgCell1           = 0xD4,
        Cell4              = 0xD5,
        Cell3              = 0xD6,
        Cell2              = 0xD7,
        Cell1              = 0xD8,
        CellX              = 0xD9,
        Batt               = 0xDA,
        AtQResidual        = 0xDC,
        AtTTE              = 0xDD,
        AtAvSOC            = 0xDE,
        AtAvCap            = 0xDF,

        CmdRegister        = 0x60,

        /* Non-volatile memory registers */
        nVEmpty            = 0x9E,
        nFullCapNom        = 0xA5,
        nFullCapRep        = 0xA9,
        nMaxMinCurr        = 0xAB,
        nMaxMinVolt        = 0xAC,
        nMaxMinTemp        = 0xAD,
        nConfig            = 0xB0,
        nDesignCap         = 0xB3,
        nPackCfg           = 0xB5,
        nNVCfg0            = 0xB8,
        nNVCfg1            = 0xB9,
        nNVCfg2            = 0xBA,
        nVAlrtTh           = 0xC0,
        nTAlrtTh           = 0xC1,
        nSAlrtTh           = 0xC2,
        nIAlrtTh           = 0xC3,
        nTGain             = 0xCA,
        nTOff              = 0xCB
    };


    MAX17201(I2C* i2c, PinName interruptPin);
    MAX17201(I2C* i2c);

    /*!
     *  Configure the gauge
     *
     *  \param number_of_cells : default value is 1
     *  \param design_capacity : capacity (mAh), default is 800mAh
     *  \param empty_voltage : voltage (V) bellow which battery is considered empty, default is 3.1V
     *  \param use_external_thermistor1 : default is false
     *  \param use_external_thermistor2 : default is false (in case both are false, will use internal thermistor)
     *  \return true on success, false on failure
     */
    bool configure(uint8_t number_of_cells = 1, uint16_t design_capacity = 800, float empty_voltage = 3.1,
            bool use_external_thermistor1 = false, bool use_external_thermistor2 = false);

    /*!
     *  Get battery state of charge (%)
     *
     *  \return Percentage of battery remaining
     */
    float state_of_charge();

    /*!
     *  Get delivered current
     *
     *  \return current (mA)
     */
    double current();

    /*!
     *  Get average current for the last 5.625 s
     *
     *  \return average current (mA)
     */
    double average_current();

    /*!
     *  Get maximum current since device reset
     *
     *  \return maximum current (mA)
     */
    float max_current();

    /*!
     *  Get minimum current since device reset
     *
     *  \return minimum current (mA)
     */
    float min_current();

    /*!
     *  Get cell voltage
     *  If more than one cell are used this is the lowest voltage of all cells
     *
     *  \return cell voltage
     */
    double cell_voltage();

    /*!
     *  Get average cell voltage for the last 45 s
     *
     *  \return average cell voltage
     */
    double average_cell_voltage();

    /*!
     *  Get maximum cell voltage since device reset
     *
     *  \return maximum cell voltage
     */
    float max_cell_voltage();

    /*!
     *  Get minimum cell voltage since device reset
     *
     *  \return minimum cell voltage
     */
    float min_cell_voltage();

    /*!
     *  Remaining time until battery is full
     *
     *  \return time to full (seconds)
     */
    float time_to_full();

    /*!
     *  Get remaining time until battery is empty
     *
     *  \return time to empty (seconds)
     */
    float time_to_empty();

    /*!
     *  Get the remaining capacity of the battery
     * 
     *  \return remaining capacicity (mAh)
     */
    float reported_capacity();

    /*!
     *  Get the full capacity of the battery computed by the gauge algorithm
     *  This is not the design capacity of the battery as the one given in the 
     *  \ref configure() function. Cell ages, temperature, cumber of cycles etc..
     *  are taken into acount to compute the full capacity
     * 
     *  \return full capacity (mAh)
     */
    float full_capacity();

    /*!
     *  Get the design capacity of the battery returned by the gauge
     *  This the capacity of the battery given in the
     *  \ref configure() function. Cell ages, temperature, cumber of cycles etc..
     *  are not taken into account to compute the design capacity
     *
     *  \return design capacity (mAh)
     */
    float design_capacity();

    /*!
     *  Get temperature of the enabled thermistor
     *  If more than one thermistor are used, it is an average the used thermistor
     * 
     *  \return temperature (°C)
     */
    float temperature();

    /*!
     *  Get the average temperature for a configurable period (default is 1.5min)
     *
     *  \return The average temperature
     */
    float average_temperature();

    /*!
     *  Get the maximum temperature since device reset
     *
     *  \return Maximum temperature
     */
    int8_t max_temperature();

    /*!
     *  Get the minimum temperature since device reset
     *
     *  \return minimum temperature
     */
    int8_t min_temperature();

    /*!
     *  Get the cell age in hours
     * 
     *  \return cell age (hours)
     */
    float age();

    /*!
     *  Number of cycle completed (1 cycle is equivalent to 1 full charge and 1 full discharge)
     *
     *  \return number of cycle
     */
    float cycle_count();

    /*!
     *  Set alerts on minimum and maximum current
     *
     *  \param max_current_threshold (mA)
     *  \param min_current_threshold (mA)
     */
    void set_current_alerts(float max_current_threshold, float min_current_threshold);

    /*!
     *  Set alerts on minimum and maximum voltage
     *
     *  \param max_voltage_threshold (V)
     *  \param min_voltage_threshold (V)
     */
    void set_voltage_alerts(float max_voltage_threshold, float min_voltage_threshold);

    /*!
     *  Set alerts on minimum and maximum battery percentage
     *
     *  \param max_soc_threshold (%)
     *  \param min_soc_threshold (%)
     */
    void set_state_of_charge_alerts(uint8_t max_soc_threshold, uint8_t min_soc_threshold);

    /*!
     *  Enable alerts on voltage, current and state of charge
     */
    void enable_alerts();

    /*!
     *  Disable alerts on voltage, current and state of charge
     */
    void disable_alerts();

    /*!
     *  Thoose are the gain and offset, specific to thermistor used, that the gauge
     *  algorithm will use to copute temperature
     *  See the gauge datasheet for more informations about the computations
     */
    void configure_thermistor(uint16_t gain, uint16_t offset);

    /*!
     *  Reset registers and gauge algorithm without loading the non-volatile memory backup
     */
    void restart_firmware();

    /*!
     *  Full reset of the gauge with with registers restoration from the non-volatile memory backup if enable
     */
    void reset();

    /*!
     *  Get the number of remaining updates of the Nonvolatile memory
     *
     *  \return The number of remaining updates
     */
    uint8_t remaining_writes();

    /*!
     *  Alert related functions
     */
    void handle_alert();

private:

    /*!
     *  Configure the empty voltage used by ModelGauge m5 algorithm
     *                   _____________________________________________________________________________________
     *  Register format |_D15_|_D14_|_D13_|_D12_|_D11_|_D10_|_D9_|_D8_|_D7_|_D6_|_D5_|_D4_|_D3_|_D2_|_D1_|_D0_|
     *                  |_____________________VEmpty_______________________|___________VRecovery______________|
     *  VEmpty is 10mv per LSB
     *  VRecovery is 40mv per LSB
     *
     *  \param VEmpty The empty voltage to use in Volts
     */
    void set_empty_voltage(float empty_voltage);

    /*!
     *  Set battery capacity
     *
     *  \param battery capacity (mAh)
     */
    void set_design_capacity(uint16_t design_capacity);

    int i2c_read_register(RegisterAddress address, uint16_t* value);
    int i2c_read_register(RegisterAddress address, int16_t* value);
    int i2c_set_register(RegisterAddress address, uint16_t value);
    int i2c_set_register(RegisterAddress address, int16_t value);

    I2C* _i2c;
    I2CAddress _i2cAddress;
    InterruptIn _interruptPin;

};

} // namespace sixtron

#endif // MAX17201_H
