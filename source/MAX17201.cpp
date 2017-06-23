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

#define R_SENSE			0.010 //Value of the sense resistor

#define TO_PERCENTAGE	(1./256)
#define TO_CAPACITY		(0.005/R_SENSE)			// mAh
#define TO_VOLTAGE		0.078125				// mV
#define TO_CURRENT		(1.5625/R_SENSE*1000)	// µA
#define TO_TEMP			(1./256)				// °C
#define TO_RESISTANCE	(1./4096)				// Ω
#define TO_SECONDS		5.625					// s

MAX17201::MAX17201(I2C* i2c, int hz, PinName interruptPin):
	_i2cAddress(I2CAddress::ModelGaugeM5Address), _interruptPin(interruptPin)
{
	_i2c = i2c;
	_i2c->frequency(hz);
}

float MAX17201::get_state_of_charge()
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

double MAX17201::get_current()
{
	double current;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::Current, &value);

	current = value*TO_CURRENT;
	return current;
}

double MAX17201::get_average_current()
{
	double current;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::AvgCurrent, &value);

	current = value*TO_CURRENT;
	return current;
}

double MAX17201::get_maximum_current()
{
	double current;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::MaxMinCurr, &value);

	current = ((value & 0xFF00) >> 8) *0.40/R_SENSE;
	return current;
}

double MAX17201::get_minimum_current()
{
	double current;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::MaxMinCurr, &value);

	current = (value & 0xFF) *0.40/R_SENSE;
	return current;
}

double MAX17201::get_VCell()
{
	double VCell;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::VCell, &value);

	VCell = value*TO_VOLTAGE;
	return VCell;
}

double MAX17201::get_capacity()
{
	double cap;
	uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::RepCap, &value);

	cap = value*TO_CAPACITY;
	return cap;
}

double MAX17201::get_full_capacity()
{
	static double cap;
	static uint16_t value;

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_read_register(RegisterAddress::FullCapRep, &value);

	cap = value*TO_CAPACITY;
	return cap;
}

float MAX17201::get_time_to_empty()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::TTE, &value);

	float time_to_empty = value*TO_SECONDS;
	return time_to_empty;
}

float MAX17201::get_time_to_full()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::TTF, &value);

	float time_to_full = value*TO_SECONDS;
	return time_to_full;
}

float MAX17201::get_temperature()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::Temp, &value);

	float temperature = value*TO_TEMP;
	return temperature;
}

/** Get the average temperature for a configurable period (default is 1.5min)
 *
 * @returns
 *      The average temperature
 */
float MAX17201::get_average_temperature()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::AvgTA, &value);

	float avg_temperature = value*TO_TEMP;
	return avg_temperature;
}

int8_t MAX17201::get_max_temperature()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::MaxMinTemp, &value);

	int8_t max_temperature = value & 0xFF00 >> 8;
	return max_temperature;
}

int8_t MAX17201::get_min_temperature()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::MaxMinTemp, &value);

	int8_t min_temperature = value & 0xFF;
	return min_temperature;
}

float MAX17201::get_age()
{
	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	uint16_t value;
	i2c_read_register(RegisterAddress::TimerH, &value);

	float age = value * 3.4;
	return age;
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
	data = (static_cast<uint16_t>(VEmpty*100 & 0x1FF) << 7) | static_cast<uint8_t>((VEmpty+0.5)*25 & 0x7F);

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_set_register(RegisterAddress::VEmpty, data);
}

void MAX17201::set_capacity(uint16_t cap)
{
	uint16_t data;
	data = static_cast<uint16_t>(cap/TO_CAPACITY);

	if (_i2cAddress != I2CAddress::ModelGaugeM5Address){
		_i2cAddress = I2CAddress::ModelGaugeM5Address;
	}

	i2c_set_register(RegisterAddress::DesignCap, data);
}

void MAX17201::handle_alert()
{

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
