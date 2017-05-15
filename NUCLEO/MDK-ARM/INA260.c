/**
 * @Description: INA260 I2C read/write functions.
 * @Author: David Tu
 * @Modified: May 15, 2017
 */

#include "stm32l4xx_hal.h"
#include "INA260.h"

/* Read data via I2C */
HAL_StatusTypeDef i2c_read( I2C_HandleTypeDef * hi2c, uint8_t device_address, uint8_t register_address, uint8_t * out_data ) {

	HAL_StatusTypeDef error;
	
	/* Send address */
	if( ( error = HAL_I2C_Master_Transmit( hi2c, device_address, &register_address, 1, 1000 ) ) != HAL_OK ) {
		
		/* Handle transmit error */
		return error;
	}
	
	/* Receive bytes */
	if( ( error = HAL_I2C_Master_Receive( hi2c, device_address, out_data, 1, 1000 ) ) != HAL_OK ) {
		
		/* Handle receive error */
		return error;
	}
	
	return HAL_OK;
}

/* Transmit data via I2C */
HAL_StatusTypeDef i2c_write( I2C_HandleTypeDef * hi2c, uint8_t device_address, uint8_t register_address, uint8_t write_data ) {
	
	/* Format data array to send */
	uint8_t t_data[2];
	t_data[0] = register_address;
	t_data[1] = write_data;
	
	HAL_StatusTypeDef error;
	
	/* Send data */
	if( ( error = HAL_I2C_Master_Transmit( hi2c, device_address, t_data, 2, 1000 ) ) != HAL_OK ) {
		
		/* Handle transmit error */
		return error;
	}
	
	return HAL_OK;
}

/* Gets the I2C address of a specific chip on the bus */
unsigned char getAddress( int chip_index ) {

  /* Map each address to an index in an array, and get the chip to read from */
  const unsigned char address_list[] = { ADDRESS_LIST };
  return address_list[ chip_index ];
}

/* Reads in current value from specific INA260 chip on I2C bus */
unsigned int getCurrent( int chip_index ) {

  /* Get the address of the chip we want to read from */
  unsigned char address = getAddress( chip_index );

  /* To record data from i2c bus */
  unsigned int data = 0;

  /* Read / write buffers */
  char read_buf[2];
  char write_buf = 0x01;

	return data;
}

/* Reads in voltage value from specific INA260 chip on I2C bus */
unsigned int getVoltage( int chip_index ) {

  /* Get the address of the chip we want to read from */
  unsigned char address = getAddress( chip_index );

  /* To record data from i2c bus */
  unsigned int data = 0;

  /* Read / write buffers */
  char read_buf[2];
  char write_buf = 0x01;

	return data;
}
