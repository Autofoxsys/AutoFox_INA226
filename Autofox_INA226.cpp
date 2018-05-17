/*
Library to facilitate use of the INA226 Voltage, Current & Power monitor
from Texas Intruments.
See spec. here: http://www.ti.com/lit/ds/symlink/ina226.pdf.

Copyright [2018] [AutoFox] autofoxsys@gmail.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

This library is for Arduino.

In developing this library, besides reading the INA226 spec, I consulted
the following code:

* Linux drivers for the INA family (written in C)
* https://github.com/SV-Zanshin/INA226 (most useful)

PLATFORM DEPENDENCE
Embodied in 3 functions related to I2C reading/writing

*/

#include "Autofox_INA226.h"
#include <Wire.h>
#include <math.h>


//=============================================================================
//Some helper macros for this source file

#define CALL_FN(fn); { status s = fn; if(fn != OK){return s;} }
#define CHECK_INITIALIZED(); if(!mInitialized) return NOT_INITIALIZED;

//=============================================================================


AutoFox_INA226::AutoFox_INA226()
{
	mInitialized = false;
	mI2C_Address = INA226_DefaultSettings::INA226_DEFAULT_I2C_ADDRESS;
	mConfigRegister = 0;
	mCalibrationValue = 0;
	mCurrentMicroAmpsPerBit = 0;
	mPowerMicroWattPerBit = 0;
}

//----------------------------------------------------------------------------
status AutoFox_INA226::Init(uint8_t aI2C_Address, double aShuntResistor_Ohms, double aMaxCurrent_Amps)
{
	mInitialized = false;

	//Check if there's a device (any I2C device) at the specified address.
	CALL_FN( CheckI2cAddress(aI2C_Address) );

	//Good so far, check that it's an INA226 device at the specified address.
	uint16_t theINA226_ID;
	CALL_FN( ReadRegister(INA226_Registers::INA226_MANUFACTURER_ID, theINA226_ID) );
	if(theINA226_ID != INA226_HardCodedChipConst::INA226_MANUFACTURER_ID){
		return INA226_TI_ID_MISMATCH; //Expected to find TI manufacturer ID
	}
	CALL_FN( ReadRegister(INA226_Registers::INA226_DIE_ID, theINA226_ID) );
	if( theINA226_ID != INA226_HardCodedChipConst::INA226_DIE_ID){
		return  INA226_DIE_ID_MISMATCH; //Expected to find INA226 device ID
	}

	mI2C_Address = aI2C_Address;

	//Reset the INA226 device
	CALL_FN( WriteRegister(INA226_Registers::INA226_CONFIG, cResetCommand) );

	//Now set our own default configuration (you can redefine this constant in the header, as needed)
	CALL_FN( WriteRegister(INA226_Registers::INA226_CONFIG, INA226_DefaultSettings::INA226_CONFIG_DEFAULT) );

	//Read back the configuration register and check that it matches
	CALL_FN( ReadRegister(INA226_Registers::INA226_CONFIG, mConfigRegister) );
	if(mConfigRegister != INA226_DefaultSettings::INA226_CONFIG_DEFAULT){
		return CONFIG_ERROR;
	}

	//Finally, set up the calibration register - this will also calculate the scaling
	//factors that we must apply to the current and power measurements that we read from
	//the INA226 device.
	CALL_FN( setupCalibration(aShuntResistor_Ohms, aMaxCurrent_Amps) );

	mInitialized = true;
	return OK;
}
//----------------------------------------------------------------------------

status AutoFox_INA226::setupCalibration(double aShuntResistor_Ohms, double aMaxCurrent_Amps)
{
	// Calculate a value for Current_LSB that gives us the best resolution
	// for current measurements.  The INA266 current register is 16-bit
	// signed, max positive value is 2^15 -1 = 32767
	// If we can be sure that the current won't be more than aMaxCurrent_Amps then
	// we can calculate the Amps per bit as aMaxCurrent_Amps/32767 (rounded up to
	// to the nearest integer).
	// The value 0.00512 in the calculations below comes from the INA226 spec which
	// provides a definition of the formula that's used to calculate the calibration value.

	double theCurrentLSB = ceil( ( aMaxCurrent_Amps * 1000000.0) / (double)32767.0);
	double theCal = (double)0.00521 /  (aShuntResistor_Ohms * (theCurrentLSB/1000000.0));

	mCurrentMicroAmpsPerBit = ((int32_t)theCurrentLSB);
	mCalibrationValue = (uint16_t)theCal;
	mPowerMicroWattPerBit = mCurrentMicroAmpsPerBit * INA226_HardCodedChipConst::INA226_POWER_LSB_FACTOR;

	return WriteRegister(INA226_Registers::INA226_CALIBRATION, mCalibrationValue);
}
//----------------------------------------------------------------------------
//Check if a device exists at the specified I2C address

status AutoFox_INA226::CheckI2cAddress(uint8_t aI2C_Address)
{
	Wire.begin();
	//Check if there's a device (any I2C device) at the specified address.
	Wire.beginTransmission(aI2C_Address);
	if(Wire.endTransmission() != 0){
		return INVALID_I2C_ADDRESS;
	}
	return OK;
}

//----------------------------------------------------------------------------

status AutoFox_INA226::ReadRegister(uint8_t aRegister, uint16_t& aValue)
{
	Wire.beginTransmission(mI2C_Address);
	Wire.write(aRegister);
	Wire.endTransmission();
	if( Wire.requestFrom((int)mI2C_Address, (int)2) == 2 ){
		aValue = Wire.read();
		aValue = aValue<<8 | Wire.read();
		return OK;
	}else{
		return FAIL;
	}
}

//----------------------------------------------------------------------------
status AutoFox_INA226::WriteRegister(uint8_t aRegister, uint16_t aValue)
{
	int theBytesWriten = 0;
	Wire.beginTransmission(mI2C_Address);
	theBytesWriten += Wire.write(aRegister);
	theBytesWriten += Wire.write((aValue >> 8) & 0xFF);
	theBytesWriten += Wire.write(aValue & 0xFF);      
	Wire.endTransmission();

	return (theBytesWriten==3) ? OK : FAIL;
}
//----------------------------------------------------------------------------
int32_t AutoFox_INA226::GetShuntVoltage_uV()
{
	//The value retrieved from the INA226 for the shunt voltage
	//needs to be multipled by 2.5 to yield the value in microvolts.
	//As I don't want to use floating point multiplication I will take the value
	//divide it by 2 (shift right) and add that to 2 times the original value
	//(shift left).
	int16_t theRegisterValue;
	int32_t theResult;
	ReadRegister(INA226_Registers::INA226_SHUNT_VOLTAGE, (uint16_t&)theRegisterValue);
	theResult = (int32_t)theRegisterValue>>1;
	theResult+= (int32_t)theRegisterValue<<1;
	return theResult;
}
//----------------------------------------------------------------------------
int32_t AutoFox_INA226::GetBusVoltage_uV()
{
	uint16_t theRegisterValue;
	ReadRegister(INA226_Registers::INA226_BUS_VOLTAGE, theRegisterValue);
	return (int32_t)theRegisterValue * INA226_HardCodedChipConst::INA226_BUS_VOLTAGE_LSB;
}

//----------------------------------------------------------------------------
int32_t AutoFox_INA226::GetCurrent_uA()
{
	int16_t theRegisterValue; // signed register, result in mA
	ReadRegister(INA226_Registers::INA226_CURRENT, (uint16_t&)theRegisterValue);
	return (int32_t)theRegisterValue * mCurrentMicroAmpsPerBit;
}
//----------------------------------------------------------------------------
int32_t AutoFox_INA226::GetPower_uW()
{
	uint16_t theRegisterValue;
	int32_t theReturnValue;
	ReadRegister(INA226_Registers::INA226_POWER, theRegisterValue);
	theReturnValue = (int32_t)theRegisterValue * mPowerMicroWattPerBit;
	return theReturnValue;
}
//----------------------------------------------------------------------------
status AutoFox_INA226::Hibernate()
{
	CHECK_INITIALIZED();
	//Make a most recent copy of the configuration register, which also contains
	//The operating mode (we need a copy of this for when we come out of sleep)
	CALL_FN( ReadRegister(INA226_Registers::INA226_CONFIG, mConfigRegister) );

	//Zero out the operating more, this will put the INA226 into shutdown
	uint16_t theTempConfigValue = mConfigRegister & ~(cOperatingModeMask);

	return WriteRegister(INA226_Registers::INA226_CONFIG, theTempConfigValue);
}
//----------------------------------------------------------------------------
status AutoFox_INA226::Wakeup()
{
	CHECK_INITIALIZED();
	//Write most recent copy of the calibration register - which should restore
	//the most operating mode that was active before hibernation.  Quick check to test if by
	//any chance the last operating mode was a hibernation and in that case set to 
	//ShuntAndBusVoltageContinuous.
	uint16_t theLastOperatingMode = mConfigRegister & cOperatingModeMask;
	if(theLastOperatingMode == eOperatingMode::Shutdown ||
		theLastOperatingMode == 0){
			mConfigRegister &= ~cOperatingModeMask;
			mConfigRegister |= eOperatingMode::ShuntAndBusVoltageContinuous;
	}

	return WriteRegister(INA226_Registers::INA226_CONFIG, mConfigRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226::SetOperatingMode(enum eOperatingMode aOpMode)
{
	CHECK_INITIALIZED();
	CALL_FN( ReadRegister(INA226_Registers::INA226_CONFIG, mConfigRegister) );

	//Zero out the existing mode then OR in the new mode
	mConfigRegister  &= ~(cOperatingModeMask);
	mConfigRegister  |= (uint16_t)aOpMode;

	return WriteRegister(INA226_Registers::INA226_CONFIG, mConfigRegister);
}
//----------------------------------------------------------------------------
status  AutoFox_INA226::ConfigureAlertPinTrigger(enum eAlertTrigger aAlertTrigger, int32_t aValue, bool aLatching)
{
	uint16_t theMaskEnableRegister;

	CHECK_INITIALIZED();
	CALL_FN( ReadRegister(INA226_Registers::INA226_MASK_ENABLE, theMaskEnableRegister) );

	//Clear the current configuration for the alert pin
	theMaskEnableRegister &= ~ cAlertPinModeMask;
	//...and prepare the new alert configuration (we'll actually set it down below)
	theMaskEnableRegister |= (uint16_t)aAlertTrigger;
	if(aLatching){
		theMaskEnableRegister |= cAlertLatchingMode;
	}

	//We need to convert the value supplied (parameter) for the trigger to an INA226 register value.
	//The supplied value could be a shunt voltage, bus voltage or power reading.  All values are
	//in micro units (i.e. microvolts or microamps).
	//Remember that when we get any reading (voltage, current or power) from the INA226 we convert
	//it from the INA226's internal regisister representation (check the functions above).  We
	//need to do the inverse conversion when providing a value back to the INA226 to act as a trigger
	//value.

	int32_t theAlertValue=0;

	switch(aAlertTrigger){
	case eAlertTrigger::PowerOverLimit:
		//back convert to INA226 representation for power (in microWatts)
		theAlertValue = (aValue / mPowerMicroWattPerBit);
		break;
	case eAlertTrigger::ClearTriggers:
	case eAlertTrigger::ConversionReady:
		//not a voltage or a power, so we don't care about the tigger value
		theAlertValue = 0;
		break;
	case eAlertTrigger::ShuntVoltageOverLimit:
	case eAlertTrigger::ShuntVoltageUnderLimit:
		theAlertValue = ((double)aValue / INA226_HardCodedChipConst::INA226_SHUNT_VOLTAGE_LSB);
		break;
	case eAlertTrigger::BusVoltageOverLimit:
	case eAlertTrigger::BusVoltageUnderLimit:
		theAlertValue = (aValue / INA226_HardCodedChipConst::INA226_BUS_VOLTAGE_LSB);
		break;
	default:
		return BAD_PARAMETER;
	}


	//before we set the new config for the alert pin, set the value that will trigger the alert
	CALL_FN( WriteRegister(INA226_Registers::INA226_ALERT_LIMIT, (int16_t)theAlertValue) );
	//Now set the tigger mode.
	return WriteRegister(INA226_Registers::INA226_MASK_ENABLE, theMaskEnableRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226::ResetAlertPin()
{
	CHECK_INITIALIZED();
	uint16_t theDummyValue;
	//Reading the Mask/Enable register will reset the alert pin
	return ReadRegister(INA226_Registers::INA226_MASK_ENABLE, theDummyValue);
}
//----------------------------------------------------------------------------
status AutoFox_INA226::ResetAlertPin(enum  eAlertTriggerCause& aAlertTriggerCause )
{
	//preset the return parameter in case the function fails
	aAlertTriggerCause = eAlertTriggerCause::Unknown;
	CHECK_INITIALIZED();

	uint16_t theTriggerCause;
	//Reading the Mask/Enable register will reset the alert pin and provide us with the
	//cause of the alert
	CALL_FN( ReadRegister(INA226_Registers::INA226_MASK_ENABLE, theTriggerCause) );

	//Mask just the bit that interests us (cause of the alert)
	theTriggerCause &= cAlertCauseMask;

	//Cast the trigger cause to the return parameter type
	aAlertTriggerCause = (eAlertTriggerCause)theTriggerCause;
	return OK;
}
//----------------------------------------------------------------------------
status AutoFox_INA226::ConfigureVoltageConversionTime(int aIndexToConversionTimeTable)
{
	CHECK_INITIALIZED();

	if(aIndexToConversionTimeTable < 0 || aIndexToConversionTimeTable > cMaxConvTimeTblIdx ){
		return BAD_PARAMETER;
	}

	//Read the configuration register
	CALL_FN( ReadRegister(INA226_Registers::INA226_CONFIG, mConfigRegister) );
	//Clear the current voltage sampling time settings
	mConfigRegister &= ~(cBusVoltageConvTimeMask | cShuntVoltageConvTimeMask);
	//Set the new values
	uint16_t theMergedBusAndShuntConvTimeIndicies = 
		((uint16_t)aIndexToConversionTimeTable << cBusVoltConvTimeIdxShift) |
		((uint16_t)aIndexToConversionTimeTable << cShuntVoltConvTimeIdxShift);

	mConfigRegister |= theMergedBusAndShuntConvTimeIndicies;

	return WriteRegister(INA226_Registers::INA226_CONFIG, mConfigRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226::ConfigureNumSampleAveraging(int aIndexToSampleAverageTable)
{
	CHECK_INITIALIZED();

	if(aIndexToSampleAverageTable < 0 || aIndexToSampleAverageTable > cMaxSampleAvgTblIdx ){
		return BAD_PARAMETER;
	}

	//Read the configuration register
	CALL_FN( ReadRegister(INA226_Registers::INA226_CONFIG, mConfigRegister) );
	//Clear the current averaging value
	mConfigRegister &= ~cSampleAvgMask;
	//Set the new value
	mConfigRegister |= (aIndexToSampleAverageTable<<cSampleAvgIdxShift);

	return WriteRegister(INA226_Registers::INA226_CONFIG, mConfigRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226::Debug_GetConfigRegister(uint16_t& aConfigReg)
{
	CHECK_INITIALIZED();
	//Read the configuration register
	return ReadRegister(INA226_Registers::INA226_CONFIG, aConfigReg);
}





