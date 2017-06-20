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
#define TO_CAPACITY		(0.005/R_SENSE)		// mAh
#define TO_VOLTAGE		0.078125			// mV
#define TO_CURRENT		(1.5625/R_SENSE)	// µA
#define TO_TEMP			(1./256)			// °C
#define TO_RESISTANCE	(1./4096)			// Ω
#define TO_SECONDS		5.625

MAX17201::MAX17201(I2C* i2c, I2CAddress address, int hz):
	_i2cAddress(address)
{
	_i2c = i2c;
	_i2c->frequency(hz);
}

float MAX17201::get_state_of_charge()
{
	float SOC;
	uint16_t value;

	i2c_read_register(RegisterAddress::RepSOC, &value);

	SOC = value*TO_PERCENTAGE;
	return SOC;
}

double MAX17201::get_current()
{
	double current;
	uint16_t value;

	i2c_read_register(RegisterAddress::Current, &value);

	current = value*TO_CURRENT;
	return current;
}

double MAX17201::get_average_current()
{
	double current;
	uint16_t value;

	i2c_read_register(RegisterAddress::AvgCurrent, &value);

	current = value*TO_CURRENT;
	return current;
}

double MAX17201::get_VCell()
{
	double VCell;
	uint16_t value;

	i2c_read_register(RegisterAddress::VCell, &value);

	VCell = value*TO_VOLTAGE;
	return VCell;
}

double MAX17201::get_capacity()
{
	double cap;
	uint16_t value;

	i2c_read_register(RegisterAddress::RepCap, &value);

	cap = value*TO_CAPACITY;
	return cap;
}

double MAX17201::get_full_capacity()
{
	double cap;
	uint16_t value;

	i2c_read_register(RegisterAddress::FullCapRep, &value);

	cap = value*TO_CAPACITY;
	return cap;
}

double MAX17201::get_time_to_empty()
{
	double TTE;
	uint16_t value;

	i2c_read_register(RegisterAddress::TTE, &value);

	TTE = value*TO_SECONDS;
	return TTE;
}

double MAX17201::get_time_to_full()
{
	double TTF;
	uint16_t value;

	i2c_read_register(RegisterAddress::TTF, &value);

	TTF= value*TO_SECONDS;
	return TTF;
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
