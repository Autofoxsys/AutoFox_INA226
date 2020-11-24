/*
 * INA2226_callback.h
 *
 *  Created on: 2020. nov. 24.
 *      Author: Norbert
 */

#ifndef INA226_INA226_CALLBACK_H_
#define INA226_INA226_CALLBACK_H_

int Transmit(AutoFox_INA226* this, uint8_t* aRegister, uint16_t Size);
int Receive(AutoFox_INA226* this, uint8_t* buffer, uint16_t Size);
int Check_device(AutoFox_INA226* this, uint8_t aI2C_Address , uint32_t Trials);

#endif /* INA226_INA226_CALLBACK_H_ */
