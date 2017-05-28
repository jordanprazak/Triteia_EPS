/**
 * @Description: I2C read/write functions to read current and bus voltage values from an INA260 chip.
 * @Author: David Tu
 * @Modified: May 15, 2017
 */

#include "stm32l4xx_hal.h"
#include "INA260.h"

/* I2C1 Handle */
static I2C_HandleTypeDef I2C1Handle = {I2C1};

/* Read data via I2C */
HAL_StatusTypeDef i2c_read( I2C_HandleTypeDef * hi2c, uint8_t device_address, uint8_t register_address, uint8_t * out_data ) {

	HAL_StatusTypeDef error;
	
	/* Send address */
	if( ( error = HAL_I2C_Master_Transmit( hi2c, device_address, &register_address, 1, 1000 ) ) != HAL_TIMEOUT ) {
		
		/* Handle transmit error */
		return error;
	}
	
	/* Receive bytes */
	if( ( error = HAL_I2C_Master_Receive( hi2c, device_address, out_data, 1, 250 ) ) != HAL_TIMEOUT ) {
		
		/* Handle receive error */
		return error;
	}
	
	return HAL_OK;
}

/* Transmit data via I2C */
HAL_StatusTypeDef i2c_write( I2C_HandleTypeDef * hi2c, uint8_t device_address, uint8_t register_address, uint8_t write_data[], int num_bytes ) {
	
	/* Format data array to send */
	uint8_t t_data[1 + num_bytes];
	t_data[0] = register_address;
	
	for (int i = 0; i < num_bytes; i++ ) {
		t_data[i + 1] = write_data[i];
	}
	
	HAL_StatusTypeDef error;
	
	/* Send data */
	if( ( error = HAL_I2C_Master_Transmit( hi2c, device_address, t_data, 1 + num_bytes, 250 ) ) != HAL_TIMEOUT ) {
		
		/* Handle transmit error */
		return error;
	}
	
	return HAL_OK;
}

/* Gets the I2C address of a specific chip on the bus */
unsigned char getAddress( int chip_index ) {

  /* Map each address to an index in an array, and get the chip to read from */
  const unsigned char address_list[] = { DEVICE_ADDRESS_LIST };
  return address_list[ chip_index ];
}

/* Send configuration byte to chip (one-shot mode) */
uint16_t sendConfig( I2C_HandleTypeDef * hi2c, int chip_index ) {
	
  /* Get the address of the chip we want to read from */
	unsigned char device_address = getAddress( chip_index );
	
	/* Write buffers - send config byte to config register */
	uint8_t register_address = CONFIG_REGISTER;
	uint8_t write_buf[2];
	write_buf[0] = CONFIG_MSB;
	write_buf[1] = CONFIG_LSB;
	
	if( i2c_write( hi2c, device_address, register_address, write_buf, 2) != HAL_OK ) {
		
		/* Handle error */
		return -1;
	}
	
	return 0;
}

/* Reads in current value from specific INA260 chip on I2C bus */
uint16_t getCurrent( I2C_HandleTypeDef * hi2c, int chip_index ) {

  /* Get the address of the chip we want to read from */
  unsigned char device_address = getAddress( chip_index );

  /* Data buffers - get value from current register */
  uint8_t read_buf[2];
  uint8_t register_address = CURRENT_REGISTER;
	
	/* Read from register */
	if( i2c_read( hi2c, device_address, register_address, read_buf ) != HAL_OK ) {
		
		/* Handle error */
	}

  /* To record data from i2c bus */
  uint16_t data = (read_buf[0] << 8) | read_buf[1];
	
	return data;
}

/* Reads in voltage value from specific INA260 chip on I2C bus */
uint16_t getVoltage( I2C_HandleTypeDef * hi2c, int chip_index ) {

  /* Get the address of the chip we want to read from */
  unsigned char device_address = getAddress( chip_index );

  /* Data buffers - get value from bus voltage register */
  uint8_t read_buf[2];
  uint8_t register_address = VOLTAGE_REGISTER;
	
	/* Read from register */
	if( i2c_read( hi2c, device_address, register_address, read_buf ) != HAL_OK ) {
		
		/* Handle error */
	}

	/* To record data from i2c bus */
  uint16_t data = (read_buf[0] << 8) | read_buf[1];
	return data;
}
