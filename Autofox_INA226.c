/*
Library to facilitate use of the INA226 Voltage, Current & Power monitor
from Texas Instruments.
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

#include "Autofox_INA226_c.h"
#include "INA226_callback.h"
#include <math.h>
#if defined(__AVR__)
#include <Wire.h>
#elif defined(USE_HAL_DRIVER)
//#include "stm32f3xx_hal.h"
#endif



//=============================================================================
//Some helper macros for this source file

#define CALL_FN(fn); { status s = fn; if(fn != OK){return s;} }
#define CHECK_INITIALIZED(); if(!this->mInitialized) return NOT_INITIALIZED;

//=============================================================================

static const uint8_t    INA226_CONFIG              = 0x00;
static const uint8_t    INA226_SHUNT_VOLTAGE       = 0x01; // readonly
static const uint8_t    INA226_BUS_VOLTAGE         = 0x02; // readonly
static const uint8_t    INA226_POWER               = 0x03; // readonly
static const uint8_t    INA226_CURRENT             = 0x04; // readonly
static const uint8_t    INA226_CALIBRATION         = 0x05;
static const uint8_t    INA226_MASK_ENABLE         = 0x06;
static const uint8_t    INA226_ALERT_LIMIT         = 0x07;
static const uint8_t    INA226_MANUFACTURER_ID     = 0xFE; // readonly
static const uint8_t    INA226_DIE_ID              = 0xFF; // readonly


//=============================================================================

static const int32_t    INA226_BUS_VOLTAGE_LSB     = 1250; //1250uV per bit
//static const int32_t    INA226_SHUNT_VOLTAGE_LSB   = 2500;    //2500 nano volts per bit (=2.5uV)
static const int32_t    INA226_POWER_LSB_FACTOR    = 25;
static const uint16_t   INA226_MANUFACTURER_ID_K   = 0x5449;
static const uint16_t   INA226_DIE_ID_K            = 0x2260;
//static const uint16_t   INA226_CONFIG_RESET_VALUE  = 0x4127; // value of config reg after a reset

//=============================================================================

static const uint16_t   INA226_CONFIG_DEFAULT       = 0x4527; // Our default config reg settings
//=============================================================================
const uint16_t cResetCommand                = 0x8000;
const uint16_t cOperatingModeMask           = 0x0007;
const uint16_t cAlertPinModeMask            = 0xFC00;
const uint16_t cAlertCauseMask              = 0x001E;
const uint16_t cAlertLatchingMode           = 0x0001;
const uint16_t cSampleAvgMask               = 0x0E00;
const uint16_t cBusVoltageConvTimeMask      = 0x01C0;
const uint16_t cShuntVoltageConvTimeMask    = 0x0038;
const int      cSampleAvgIdxShift           = 9;
const int      cBusVoltConvTimeIdxShift     = 6;
const int      cShuntVoltConvTimeIdxShift   = 3;
const int      cMaxSampleAvgTblIdx          = 7;    //occupies 3 bit positions
const int      cMaxConvTimeTblIdx           = 7; //occupies 3 bit positions
//=============================================================================

void AutoFox_INA226_Constructor(AutoFox_INA226* this, void* i2c_device, uint8_t aI2C_Address)
{
	this->mInitialized = false;
	this->hi2c = i2c_device;
	this->mI2C_Address = aI2C_Address;
	this->mConfigRegister = 0;
	this->mCalibrationValue = 0;
	this->mCurrentMicroAmpsPerBit = 0;
	this->mPowerMicroWattPerBit = 0;
}

//----------------------------------------------------------------------------
status AutoFox_INA226_Init(AutoFox_INA226* this, void* i2c_device, uint8_t aI2C_Address, double aShuntResistor_Ohms, double aMaxCurrent_Amps)
{
	AutoFox_INA226_Constructor(this, i2c_device, aI2C_Address);

	//Check if there's a device (any I2C device) at the specified address.
	CALL_FN( AutoFox_INA226_CheckI2cAddress(this, aI2C_Address) );

	//Good so far, check that it's an INA226 device at the specified address.
	uint16_t theINA226_ID;
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_MANUFACTURER_ID, &theINA226_ID) );
	if(theINA226_ID != INA226_MANUFACTURER_ID_K){
		return INA226_TI_ID_MISMATCH; //Expected to find TI manufacturer ID
	}
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_DIE_ID, &theINA226_ID) );
	if( theINA226_ID != INA226_DIE_ID_K){
		return  INA226_DIE_ID_MISMATCH; //Expected to find INA226 device ID
	}

	

	//Reset the INA226 device
	CALL_FN( AutoFox_INA226_WriteRegister(this,INA226_CONFIG, cResetCommand) );

	//Now set our own default configuration (you can redefine this constant in the header, as needed)
	CALL_FN( AutoFox_INA226_WriteRegister(this,INA226_CONFIG, INA226_CONFIG_DEFAULT) );

	//Read back the configuration register and check that it matches
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_CONFIG, &(this->mConfigRegister)) );
	if(this->mConfigRegister != INA226_CONFIG_DEFAULT){
		return CONFIG_ERROR;
	}

	//Finally, set up the calibration register - this will also calculate the scaling
	//factors that we must apply to the current and power measurements that we read from
	//the INA226 device.
	CALL_FN( AutoFox_INA226_setupCalibration(this, aShuntResistor_Ohms, aMaxCurrent_Amps) );

	this->mInitialized = true;
	return OK;
}
//----------------------------------------------------------------------------

status AutoFox_INA226_setupCalibration(AutoFox_INA226* this, double aShuntResistor_Ohms, double aMaxCurrent_Amps)
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

	this->mCurrentMicroAmpsPerBit = ((int32_t)theCurrentLSB);
	this->mCalibrationValue = (uint16_t)theCal;
	this->mPowerMicroWattPerBit = this->mCurrentMicroAmpsPerBit * INA226_POWER_LSB_FACTOR;

	return AutoFox_INA226_WriteRegister(this,INA226_CALIBRATION, this->mCalibrationValue);
}
//----------------------------------------------------------------------------
//Check if a device exists at the specified I2C address

status AutoFox_INA226_CheckI2cAddress(AutoFox_INA226* this, uint8_t aI2C_Address)
{
#if defined(__AVR__)
	Wire.begin();
	//Check if there's a device (any I2C device) at the specified address.
	Wire.beginTransmission(aI2C_Address);
	if(Wire.endTransmission() == 0){
		return OK;
	}
#elif defined(USE_HAL_DRIVER)
	if(Check_device(this, (uint16_t)aI2C_Address, 10) != 0){ //Return 0 is OK
		return INVALID_I2C_ADDRESS;
	}else{
		return OK;
	}
#endif
	return INVALID_I2C_ADDRESS;
}

//----------------------------------------------------------------------------

status AutoFox_INA226_ReadRegister(AutoFox_INA226* this, uint8_t aRegister, uint16_t* aValue_p)
{
	*aValue_p = 0;

#if defined(__AVR__)
	Wire.beginTransmission(mI2C_Address);
	Wire.write(aRegister);
	Wire.endTransmission();
	if( Wire.requestFrom((int)this->mI2C_Address, (int)2) == 2 ) {
		*aValue_p = Wire.read();
		*aValue_p = *aValue_p<<8 | Wire.read();
		return OK;
	}
#elif defined(USE_HAL_DRIVER)
	uint8_t buffer[2];
	if (Transmit(this, &aRegister, 1)
			!= 0) {							//Return 0 is OK
		return FAIL;
	}
	if (Receive(this, buffer, 2) != 0) {	//Return 0 is OK
		return FAIL;
	}
	*aValue_p = buffer[0];
	*aValue_p = *aValue_p<<8 | buffer[1];
	return OK;
#endif
	return FAIL;
}

//----------------------------------------------------------------------------
status AutoFox_INA226_WriteRegister(AutoFox_INA226* this, uint8_t aRegister, uint16_t aValue)
{
#if defined(__AVR__)
	int theBytesWriten = 0;
	Wire.beginTransmission(this->mI2C_Address);
	theBytesWriten += Wire.write(aRegister);
	theBytesWriten += Wire.write((aValue >> 8) & 0xFF);
	theBytesWriten += Wire.write(aValue & 0xFF);
	Wire.endTransmission();
	return (theBytesWriten==3) ? OK : FAIL;
#elif defined(USE_HAL_DRIVER)
	uint8_t buffer[3];
	buffer[0] = aRegister;
	buffer[1] = (uint8_t) ((aValue >> 8) & 0xFF);
	buffer[2] = (uint8_t) (aValue & 0xFF);
	if (Transmit(this, buffer, 3)
			!= 0) {				//Return 0 is OK
		return FAIL;
	}
	return OK;
#endif
	return FAIL;
}
//----------------------------------------------------------------------------
int32_t AutoFox_INA226_GetShuntVoltage_uV(AutoFox_INA226* this)
{
	//The value retrieved from the INA226 for the shunt voltage
	//needs to be multiplied by 2.5 to yield the value in microvolts.
	//As I don't want to use floating point multiplication I will take the value
	//divide it by 2 (shift right) and add that to 2 times the original value
	//(shift left).
	int16_t theRegisterValue=0;
	int32_t theResult;
	AutoFox_INA226_ReadRegister(this,INA226_SHUNT_VOLTAGE, (uint16_t*)&theRegisterValue);
	theResult = (int32_t)theRegisterValue>>1;
	theResult+= (int32_t)theRegisterValue<<1;
	return theResult;
}
//----------------------------------------------------------------------------
int32_t AutoFox_INA226_GetBusVoltage_uV(AutoFox_INA226* this)
{
	uint16_t theRegisterValue=0;
	AutoFox_INA226_ReadRegister(this,INA226_BUS_VOLTAGE, &theRegisterValue);
	return (int32_t)theRegisterValue * INA226_BUS_VOLTAGE_LSB;
}

//----------------------------------------------------------------------------
int32_t AutoFox_INA226_GetCurrent_uA(AutoFox_INA226* this)
{
	int16_t theRegisterValue=0; // signed register, result in mA
	AutoFox_INA226_ReadRegister(this,INA226_CURRENT, (uint16_t*)&theRegisterValue);
	return (int32_t)theRegisterValue * this->mCurrentMicroAmpsPerBit;
}
//----------------------------------------------------------------------------
int32_t AutoFox_INA226_GetPower_uW(AutoFox_INA226* this)
{
	uint16_t theRegisterValue=0;
	int32_t theReturnValue;
	AutoFox_INA226_ReadRegister(this,INA226_POWER, &theRegisterValue);
	theReturnValue = (int32_t)theRegisterValue * this->mPowerMicroWattPerBit;
	return theReturnValue;
}
//----------------------------------------------------------------------------
status AutoFox_INA226_Hibernate(AutoFox_INA226* this)
{
	CHECK_INITIALIZED();
	//Make a most recent copy of the configuration register, which also contains
	//The operating mode (we need a copy of this for when we come out of sleep)
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_CONFIG, &(this->mConfigRegister)) );

	//Zero out the operating more, this will put the INA226 into shutdown
	uint16_t theTempConfigValue = this->mConfigRegister & ~(cOperatingModeMask);

	return AutoFox_INA226_WriteRegister(this,INA226_CONFIG, theTempConfigValue);
}
//----------------------------------------------------------------------------
status AutoFox_INA226_Wakeup(AutoFox_INA226* this)
{
	CHECK_INITIALIZED();
	//Write most recent copy of the calibration register - which should restore
	//the most operating mode that was active before hibernation.  Quick check to test if by
	//any chance the last operating mode was a hibernation and in that case set to 
	//ShuntAndBusVoltageContinuous.
	uint16_t theLastOperatingMode = this->mConfigRegister & cOperatingModeMask;
	if(theLastOperatingMode == Shutdown ||
		theLastOperatingMode == 0){
			this->mConfigRegister &= ~cOperatingModeMask;
			this->mConfigRegister |= ShuntAndBusVoltageContinuous;
	}

	return AutoFox_INA226_WriteRegister(this,INA226_CONFIG, this->mConfigRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226_SetOperatingMode(AutoFox_INA226* this, enum eOperatingMode aOpMode)
{
	CHECK_INITIALIZED();
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_CONFIG, &(this->mConfigRegister)) );

	//Zero out the existing mode then OR in the new mode
	this->mConfigRegister  &= ~(cOperatingModeMask);
	this->mConfigRegister  |= (uint16_t)aOpMode;

	return AutoFox_INA226_WriteRegister(this,INA226_CONFIG, this->mConfigRegister);
}
//----------------------------------------------------------------------------
status  AutoFox_INA226_ConfigureAlertPinTrigger(AutoFox_INA226* this, enum eAlertTrigger aAlertTrigger, int32_t aValue, bool aLatching)
{
	uint16_t theMaskEnableRegister;

	CHECK_INITIALIZED();
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_MASK_ENABLE, &theMaskEnableRegister) );

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
	case PowerOverLimit:
		//back convert to INA226 representation for power (in microWatts)
		theAlertValue = (aValue / this->mPowerMicroWattPerBit);
		break;
	case ClearTriggers:
	case ConversionReady:
		//not a voltage or a power, so we don't care about the trigger value
		theAlertValue = 0;
		break;
	case ShuntVoltageOverLimit:
	case ShuntVoltageUnderLimit:
		//theAlertValue = ((double)aValue / ((double)(INA226_SHUNT_VOLTAGE_LSB)/1000.0));
		theAlertValue = (aValue<<1)/5; //same as aValue/2.5
		break;
	case BusVoltageOverLimit:
	case BusVoltageUnderLimit:
		theAlertValue = (aValue / INA226_BUS_VOLTAGE_LSB);
		break;
	default:
		return BAD_PARAMETER;
	}


	//before we set the new config for the alert pin, set the value that will trigger the alert
	CALL_FN( AutoFox_INA226_WriteRegister(this,INA226_ALERT_LIMIT, (int16_t)theAlertValue) );
	//Now set the trigger mode.
	return AutoFox_INA226_WriteRegister(this,INA226_MASK_ENABLE, theMaskEnableRegister);
}
//----------------------------------------------------------------------------
//status AutoFox_INA226_ResetAlertPin(AutoFox_INA226* this)
//{
//	CHECK_INITIALIZED();
//	uint16_t theDummyValue;
//	//Reading the Mask/Enable register will reset the alert pin
//	return AutoFox_INA226_ReadRegister(this,INA226_MASK_ENABLE, theDummyValue);
//}
//----------------------------------------------------------------------------
status AutoFox_INA226_ResetAlertPin(AutoFox_INA226* this, enum  eAlertTriggerCause* aAlertTriggerCause_p )
{
	//preset the return parameter in case the function fails
	*aAlertTriggerCause_p = Unknown;
	CHECK_INITIALIZED();

	uint16_t theTriggerCause;
	//Reading the Mask/Enable register will reset the alert pin and provide us with the
	//cause of the alert
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_MASK_ENABLE, &theTriggerCause) );

	//Mask just the bit that interests us (cause of the alert)
	theTriggerCause &= cAlertCauseMask;

	//Cast the trigger cause to the return parameter type
	*aAlertTriggerCause_p = theTriggerCause;
	return OK;
}
//----------------------------------------------------------------------------
status AutoFox_INA226_ConfigureVoltageConversionTime(AutoFox_INA226* this, int aIndexToConversionTimeTable)
{
	CHECK_INITIALIZED();

	if(aIndexToConversionTimeTable < 0 || aIndexToConversionTimeTable > cMaxConvTimeTblIdx ){
		return BAD_PARAMETER;
	}

	//Read the configuration register
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_CONFIG, &(this->mConfigRegister)) );
	//Clear the current voltage sampling time settings
	this->mConfigRegister &= ~(cBusVoltageConvTimeMask | cShuntVoltageConvTimeMask);
	//Set the new values
	uint16_t theMergedBusAndShuntConvTimeIndicies = 
		((uint16_t)aIndexToConversionTimeTable << cBusVoltConvTimeIdxShift) |
		((uint16_t)aIndexToConversionTimeTable << cShuntVoltConvTimeIdxShift);

	this->mConfigRegister |= theMergedBusAndShuntConvTimeIndicies;

	return AutoFox_INA226_WriteRegister(this,INA226_CONFIG, this->mConfigRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226_ConfigureNumSampleAveraging(AutoFox_INA226* this, int aIndexToSampleAverageTable)
{
	CHECK_INITIALIZED();

	if(aIndexToSampleAverageTable < 0 || aIndexToSampleAverageTable > cMaxSampleAvgTblIdx ){
		return BAD_PARAMETER;
	}

	//Read the configuration register
	CALL_FN( AutoFox_INA226_ReadRegister(this,INA226_CONFIG, &(this->mConfigRegister)) );
	//Clear the current averaging value
	this->mConfigRegister &= ~cSampleAvgMask;
	//Set the new value
	this->mConfigRegister |= (aIndexToSampleAverageTable<<cSampleAvgIdxShift);

	return AutoFox_INA226_WriteRegister(this,INA226_CONFIG, this->mConfigRegister);
}
//----------------------------------------------------------------------------
status AutoFox_INA226_Debug_GetConfigRegister(AutoFox_INA226* this, uint16_t* aConfigReg_p)
{
	CHECK_INITIALIZED();
	//Read the configuration register
	return AutoFox_INA226_ReadRegister(this,INA226_CONFIG, aConfigReg_p);
}





