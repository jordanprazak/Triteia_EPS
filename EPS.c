#include <stdio.h>
#include <stdint.h>
#include "EPS.h"
#include <linux/i2c-dev.h>



int main()
{
	printf("start\n");
	if (initialize_mmap_adc() != 0) {	
		printf("adc_init_failure\n");
	}
	else printf("adc initialized\n");
	int temp;
	temp = mmap_adc_read_raw(0);
	printf("%i\n,",temp);
	
	gpio_set_dir( 67, 1);
	//get_iv_INA3221( 0, 0);
	while(1) {
		gpio_set_value( 67, 1);
		sleep(1);
		gpio_set_value( 67, 0);
		sleep(1);
	}
	return 0;
}



