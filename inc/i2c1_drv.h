/*
 * i2c1_comm.h
 *
 *  Created on: Jan 28, 2018
 *      Author: abel
 */

#ifndef I2C1_DRV_H_
#define I2C1_DRV_H_

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"


void init_i2c_bus_config (void);
void init_i2c_gpio (void);
void I2C_Transmit (uint8_t data, uint8_t hw_address);

#endif /* I2C1_DRV_H_ */
