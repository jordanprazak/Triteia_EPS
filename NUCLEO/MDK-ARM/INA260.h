/**
 * @Description: INA260 I2C read/write function declarations and constants.
 * @Author: David Tu
 * @Modified: May 15, 2017
 */
#ifndef INA260_H
#define INA260_H

#include "stm32l4xx_hal.h"

/* See INA260 datasheet Table 2 (p. 18) for how to hardwire them */
#define DEVICE_ADDRESS_LIST 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49

#define CONFIG_REGISTER 0x00
#define CURRENT_REGISTER 0x01
#define VOLTAGE_REGISTER 0x02

/***** Configuration Register Fields *****/
#define RESET_BIT 0
#define AVG_NUM_SAMPLES 4
#define CONVERSION_TIME_INA260 1100
#define OPERATING_MODE 0x03 // Triggered

/*
  bit15 = Reset (Keep Low)
  bit14-12 = Unused
  bit11-9 = channel average count
  bit8-6 = bus voltage conversion time
  bit5-3 = shunt voltage conversion time
  bit2-0 = operating mode
*/
#define CONFIG_MSB 0x63 /* 0110 0011 */
#define CONFIG_LSB 0x23 /* 0010 0011 */

/* Generic read/write functions for INA260. Returns status value. */
// Normal Linux i2c dev: read/write( file handler, R/W buffer, # bytes );
//HAL_StatusTypeDef i2c_read( I2C_HandleTypeDef * hi2c, uint8_t device_address, uint8_t register_address, uint8_t * out_data );
//HAL_StatusTypeDef i2c_write( I2C_HandleTypeDef * hi2c, uint8_t device_address, uint8_t register_address, uint8_t write_data );

/* Get an iv value from specific INA260 chip on I2C bus. Return data. */
unsigned char getAddress( int chip_index );
uint16_t getCurrent( int chip_index );
uint16_t getVoltage( int chip_index );
uint16_t sendConfig( int chip_index );

#endif /* INA260_H */
