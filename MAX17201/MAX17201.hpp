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

class MAX17201
{
public:
	/* I2C addresses */
	enum class I2CAdress : char {
		GaugeDataAndConfigAddress	= 0x6C,
		NonVolatileMemoryAddress	= 0x16, // WARNING : This memory is limited to 7 writes. Then it is permanently locked.
		SBSDataAddress				= 0x16
	};

    /* Represents the different alert flags for the MAX17048 */
    enum class AlertFlags : char { // short ?
        ALERT_RI = (1 << 0),  /**< Reset indicator */
        ALERT_VH = (1 << 1),  /**< Voltage high alert */
        ALERT_VL = (1 << 2),  /**< Voltage low alert */
        ALERT_VR = (1 << 3),  /**< Voltage reset alert */
        ALERT_HD = (1 << 4),  /**< SOC low alert */
        ALERT_SC = (1 << 5)   /**< SOC change alert */
    };

    enum class RegisterAdress : char {
    	/* ModelGauge m5 Registers */
    	Status			= 0x00,
    	VAlrtTh			= 0x01,
		TAlrtTh			= 0x02,
		SAlrtTh			= 0x03,
		AtRate			= 0x04,
    	RepCap			= 0x05,
    	RepSOC			= 0x06,
		Age				= 0x07,
		Temp			= 0x08,
		VCell			= 0x09,
		Current			= 0x0A,
		AvgCurrent		= 0x0B,
		QResidual		= 0x0C,
		MixSOC			= 0x0D,
		AvSOC			= 0x0E,
		MixCap			= 0x0F,

		FullCap			= 0x10,
		TTE				= 0x11,
		QRTable00		= 0x12,
		FullSocThr		= 0x13,
		RCell			= 0x14,
		RFast			= 0x15,
		AvgTA			= 0x16,
		Cycles			= 0x17,
		DesignCap		= 0x18,
		AvgVCell		= 0x19,
		MaxMinTemp		= 0x1A,
		MaxMinVolt		= 0x1B,
		MaxMinCurr		= 0x1C,
		Config			= 0x1D,
		IChgTerm		= 0x1E,
		AvCap			= 0x1F,

		TTF				= 0x20,
		DevName			= 0x21,
		QRTable10		= 0x22,
		FullCapNom		= 0x23,
		AIN0			= 0x27,
		LearnCfg		= 0x28,
		FilterCfg		= 0x29,
		RelaxCfg		= 0x2A,
		MiscCfg			= 0x2B,
		TGain			= 0x2C,
		Toff			= 0x2D,
		CGain			= 0x2E,
		COff			= 0x2F,

		QRTable20		= 0x32,
		FullCapRep		= 0x35,
		IAvgEmpty		= 0x36,
		RComp0			= 0x38,
		TempCo			= 0x39,
		VEmpty			= 0x3A,
		Fstat			= 0x3D,
		Timer			= 0x3E,
		ShdnTimer		= 0x3F,

		QRTable30		= 0x42,
		dQAcc			= 0x45,
		dPAcc			= 0x46,
		VFRemCap		= 0x4A,
		QH				= 0x4D,

		Status2			= 0xB0,
		IAlrtTh			= 0xB4,
		VShdnCfg		= 0xB8,
		AgeForecast		= 0xB9,
		HibCfg			= 0xBA,
		Config2			= 0xBB,
		VRipple			= 0xBC,
		PackCfg			= 0xBD,
		TimerH			= 0xBE,

		AvgCell4		= 0xD1,
		AvgCell3		= 0xD2,
		AvgCell2		= 0xD3,
		AvgCell1		= 0xD4,
		Cell4			= 0xD5,
		Cell3			= 0xD6,
		Cell2			= 0xD7,
		Cell1			= 0xD8,
		CellX			= 0xD9,
		Batt			= 0xDA,
		AtQResidual		= 0xDC,
		AtTTE			= 0xDD,
		AtAvSOC			= 0xDE,
		AtAvCap			= 0xDF
    };


	MAX17201(I2C* i2c, int hz = 4000000);

private:

	unsigned short i2c_read_register(RegisterAdress);
	void i2c_set_register(RegisterAdress, unsigned short data);

	I2C* _i2c;
	static char _addr;

};

#endif // MAX17201_H
