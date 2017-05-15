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
*config_INA260
*Config command for the INA260
*This command sets the configuration for the INA260 IV monitor, it is also 
*required to initiate a measurement if the chip is in one shot mode                                                                       
*******************************************************************************/

int config_INA260(int chip_select) {
	//lookup table for the different I2C addresses
    int addresslist[] = {ADDRESS_LIST_INA260};
    //set the address based on which chip the user would like a measurement from.                                
    int address;
	unsigned int output;
    address = addresslist[chip_select];
    int I2C_fdef;
    char *filename = "/dev/i2c-2";
    
    I2C_fdef = open(filename, O_RDWR);
    if (I2C_fdef < 0) {
        printf("I2C file open error\n");
        //exit(-1);
    }
    
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure\n");
        //exit(-1);
    }
	char write_buf[2];
	write_buf[0] = 0x00;
	write_buf[1] = INA260_CONFIG_MSB;
	write_buf[2] = INA260_CONFIG_LSB;
	
	if (write(I2C_fdef, write_buf, 3) != 3) {
        printf("error: I2C config failed\n");
        //exit(-1);
    }
	if(close(I2C_fdef) != 0)
		printf("I2C file close failure\n");
	
	return 0;
}

/*******************************************************************************                                                                              
*get_iv_INA260
*Get a current and voltage measurement from the INA260 current monitor
*Returns an unsigned int with voltage in the lower byte and current in the *
*upper byte                                                                       
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
        printf("I2C file open error\n");
        //exit(-1);
    }
    
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure\n");
        //exit(-1);
    }
    
    
    char read_buf[2];
	char write_buf[] = {0x01};
    if (write(I2C_fdef, write_buf, 1) != 1) {
        printf("error: I2C write failed\n");
        //exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed\n");
        //exit(-1);
    }
    output = (read_buf[0] << 8) | read_buf[1];
	write_buf[0] = 0x02;
	if (write(I2C_fdef, write_buf, 1) != 1) {
        printf("error: I2C write failed\n");
        //exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed\n");
        //exit(-1);
    }
    output =  (output << 16) + ((read_buf[0] << 8) | read_buf[1]);
	if(close(I2C_fdef) != 0)
		printf("I2C file close failure\n");
    return output;
}

/*******************************************************************************                                                                              
*config_INA3221
*Config command for the INA3221
*This command sets the configuration for the INA3221 IV monitor, it is also 
*required to initiate a measurement if the chip is in one shot mode                                                                       
*******************************************************************************/
int config_INA3221(int chip_select) {
	//lookup table for the different I2C addresses
    int addresslist[] = {ADDRESS_LIST_INA3221};
    //set the address based on which chip the user would like a measurement from.                                
    int address;
	unsigned int output;
    address = addresslist[chip_select];
    int I2C_fdef;
    char *filename = "/dev/i2c-2";
    
    I2C_fdef = open(filename, O_RDWR);
    if (I2C_fdef < 0) {
        printf("I2C file open error\n");
        //exit(-1);
    }
    
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure\n");
        //exit(-1);
    }
	char write_buf[2];
	write_buf[0] = 0x00;
	write_buf[1] = INA3221_CONFIG_MSB;
	write_buf[2] = INA3221_CONFIG_LSB;
	
	if (write(I2C_fdef, write_buf, 3) != 3) {
        printf("error: I2C config failed\n");
        //exit(-1);
    }
	if(close(I2C_fdef) != 0)
		printf("I2C file close failure\n");
	return 0;
}

/*******************************************************************************                                                                              
*get_iv_INA3221
*Config command for the INA3221
*This command sets the configuration for the INA260 IV monitor, it is also 
*required to initiate a measurement if the chip is in one shot mode                                                                       
*******************************************************************************/
unsigned int get_iv_INA3221( int chip_select, int channel_select) {
    //lookup table for the different I2C addresses
    int addresslist[] = {ADDRESS_LIST_INA3221};
    //set the address based on which chip the user would like a measurement from.                                
    int address;
	unsigned int output;
    address = addresslist[chip_select];
    int I2C_fdef;
    char *filename = "/dev/i2c-2";
    
    I2C_fdef = open(filename, O_RDWR);
    if (I2C_fdef < 0) {
        printf("I2C file open error\n");
        //exit(-1);
    }
    
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure\n");
        //exit(-1);
    }
    char iv_register;
    iv_register = (char)(channel_select*2 + 1);
	
    char read_buf[2];
    if (write(I2C_fdef, &iv_register, 1) != 1) {
        printf("error: I2C reg write failed\n");
        //exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed\n");
        //exit(-1);
    }
	
    output = (read_buf[0] << 8) | read_buf[1];
	
	iv_register = iv_register + 1;
	
	if (write(I2C_fdef, &iv_register, 1) != 1) {
        printf("error: I2C reg write failed\n");
        //exit(-1);
    }
    if (read(I2C_fdef, read_buf, 2) != 2) {
        printf("error: I2C read failed\n");
        //exit(-1);
    }

    output = (output << 16) + ((read_buf[0] << 8) | read_buf[1]);
	if(close(I2C_fdef) != 0)
		printf("I2C file close failure\n");
    return output;
}
