#include "INA260.h"

unsigned char getAddress( int chip_index ) {
  /* Map each address to an index in an array, and get the chip to read from */
  const unsigned char const address_list = { ADDRESS_LIST };
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

}
