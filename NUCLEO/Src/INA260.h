#ifndef INA260_H
#define INA260_H

/* Time in microseconds for ADC shunt current and bus voltage measurements */
#define CONVERSION_TIME_INA260 1100

/* See INA260 datasheet Table 2 (p. 18) for how to hardwire them */
#define ADDRESS_LIST 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49

/*
  bit15 = Reset (Keep Low)
  bit14-12 = Unused
  bit11-9 = channel average count
  bit8-6 = bus voltage conversion time
  bit5-3 = shunt voltage conversion time
  bit2-0 = operating mode
*/
#define INA260_CONFIG_MSB 0x63 /* 0110 0011 ??? */
#define INA260_CONFIG_LSB 0x23 /* 0010 0011 ??? */

/* Generic read/write functions for INA260. Returns status value. */
// Normal Linux i2c dev: read/write( file handler, R/W buffer, # bytes );
int read( const char * read_buf, unsigned char address );
int write( char * write_buf, unsigned char address );

/* Get an iv value from specific INA260 chip on I2C bus. Returns data. */
unsigned char getAddress( int chip_index );
unsigned int getCurrent( int chip_index );
unsigned int getVoltage( int chip_index );
unsigned int getStatus( int chip_index );

#endif /* INA260_H */
