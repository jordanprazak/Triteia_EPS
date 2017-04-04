#include <stdio.h>
#include <stdint.h>
#include "EPS.h"
#include <linux/i2c-dev.h>



int main()
{
	//printf("start\n");
	/*if (initialize_mmap_adc() != 0) {	
		printf("adc_init_failure\n");
	}
	else printf("adc initialized\n");
	double temp;*/
	
	
	//reset configuration
	/* int I2C_fdef;
    char *filename = "/dev/i2c-2";
    
    I2C_fdef = open(filename, O_RDWR);
    if (I2C_fdef < 0) {
        printf("I2C file open error");
        exit(-1);
    }
    int address = 0x40;
    if( ioctl(I2C_fdef, I2C_SLAVE, address) < 0) {
        printf("I2C device instantiation failure");
        exit(-1);
    }
	unsigned char write_buf[] = {}
    if (write(I2C_fdef, &iv_register, 1) != 1) {
        printf("error: I2C reg write failed\n");
        exit(-1);
    } */
	gpio_export(PCM_IN_EN);
	gpio_set_dir(PCM_IN_EN, 1);
	gpio_set_value(PCM_IN_EN, 0);
	sleep(15);
	gpio_set_value(PCM_IN_EN,1);
	//printf("GPIO set to out\n");
	unsigned int solar0_iv;
	int16_t solar0_i;
	int16_t solar0_v;
	double scaled_i;
	double scaled_v;
	
	while(1) {
		/* config_INA260(0);
		usleep(CONVERSION_TIME_INA3221);
		solar0_iv = get_iv_INA260(0);
		solar0_i = (solar0_iv >> 16);
		solar0_v = solar0_iv;
		scaled_i = (double)solar0_i*0.00125;
		scaled_v = (double)solar0_v*0.00125;
		printf("\r scaled_i = %lf, ",scaled_i);
		printf("scaled_v = %lf",scaled_v); */
		
		config_INA3221(0);
		usleep(CONVERSION_TIME_INA3221);
		solar0_iv = get_iv_INA3221(0,2);
		solar0_i = (solar0_iv >> 16);
		solar0_v = solar0_iv;
		scaled_i = (double)solar0_i*0.04/(8*2) + INA3221_OFFSET;
		scaled_v = (double)solar0_v*0.008/8;
		printf("\r scaled_i = %lf, ",scaled_i);
		printf("scaled_v = %lf",scaled_v);
		
		
		fflush(stdout);
		//GPIO TEST
		/*
		gpio_set_value( 67, 1);
		printf("ON\n");
		usleep(700000);
		gpio_set_value( 67, 0);
		printf("OFF\n");
		usleep(700000);
		*/
		//TEMP TEST
		//temp = (double)mmap_adc_read_raw(0)*1.8*1000000/4095/994 - 273.2 + \
		TEMP_CALIBRATION;
		//printf("\r temp_voltage = %lf", temp);
		//fflush(stdout);
		usleep(100000);

	}
	return 0;
}



