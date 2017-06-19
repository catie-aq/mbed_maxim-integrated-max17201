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

MAX17201::MAX17201(I2C* i2c, I2CAddress address, int hz):
	_i2cAddress(address)
{
	_i2c = i2c;
	_i2c->frequency(hz);
}

int MAX17201::i2c_set_register(RegisterAddress address, unsigned short value)
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

int MAX17201::i2c_read_register(RegisterAddress address, unsigned short *value)
{
    static char data[2];
    data[0] = static_cast<char>(address);

    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data , 2, false) != 0) {
        return -2;
    }

    *value = (data[1] << 8) | (data[0] & 0xFF);
    return 0;
}
