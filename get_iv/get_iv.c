#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "get_iv.h"


/*******************************************************************************                                                                              
*get_iv
*Get a current and voltage measurement from the specified i2C chip.
*Returns an unsigned int with current in the lower byte and voltage in the upper
*byte                                                                       
*******************************************************************************/
unsigned int get_iv_INA260( int chip_select)
{
    //lookup table for the different I2C addresses
    unsigned char addresslist[] = {ADDRESS_LIST_INA260};
	//initialize output array
	unsigned int output = 0;
    //set the address based on which chip the user would like a measurement from.                                
    unsigned char address;
    address = addresslist[chip_select];
    
    int I2C_fdef;
    char *filename = "/dev/i2c-2";
    
    I2C_fdef = open(filename, O_RDWR);
    if (I2C_fdef < 0) {
        printf("I2C file open error");
        exit(-1);
    }
    
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure");
        exit(-1);
    }
    
    
    char read_buf[2];
	char write_buf[] = {0x01};
    if (write(I2C_fdef, write_buf, 1) != 1) {
        printf("error: I2C write failed");
        exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed");
        exit(-1);
    }
    output = (read_buf[1] << 8) | read_buf[2];
	write_buf[0] = 0x02;
	if (write(I2C_fdef, write_buf, 1) != 1) {
        printf("error: I2C write failed");
        exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed");
        exit(-1);
    }
    output =  (output << 16) + ((read_buf[1] << 8) | read_buf[2]);
	
    return output;
}

unsigned int get_iv_INA3221( int chip_select, int channel_select) {
     //lookup table for the different I2C addresses
    unsigned char addresslist[] = {ADDRESS_LIST_INA260};
    //set the address based on which chip the user would like a measurement from.                                
    unsigned char address;
	unsigned int output;
    address = addresslist[chip_select];
    
    int I2C_fdef;
    char *filename = "/dev/i2c-2";
    
    I2C_fdef = open(filename, O_RDWR);
    if (I2C_fdef < 0) {
        printf("I2C file open error");
        exit(-1);
    }
    
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure");
        exit(-1);
    }
    char iv_register;
    iv_register = (char)(channel_select*2 + 1);
    
	
    char read_buf[2];
    if (write(I2C_fdef, &iv_register, 1) != 1) {
        printf("error: I2C write failed");
        exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed");
        exit(-1);
    }
    output = (read_buf[1] << 8) | read_buf[2];
	
	iv_register = iv_register + 1;
	if (write(I2C_fdef, &iv_register, 1) != 1) {
        printf("error: I2C write failed");
        exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed");
        exit(-1);
    }
    output = (output << 16) + ((read_buf[1] << 8) | read_buf[2]);
    return output;
}
