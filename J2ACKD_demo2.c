#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include "EPS.h"

#define SRV_IP "192.168.7.1"
#define BUFLEN 1024
#define NPACK 10
#define PORT 5000


typedef struct packet_t {
		int PDM0;
		int PDM1;
		int PDM2;
		int PDM3;
		int PDM4;
		int PDM5;
		int PDM6;
		int PDM7;
		int PDM8;
		int PDM9;
		int PDM10;
		int PDM11;
		float I0;
		float I1;
		float I2;
		float I3;
		float I4;
		float I5;
		float I6;
		float I7;
		float I8;
		float I9;
		float V0;
		float V1;
		float V2;
		float V3;
		float V4;
		float V5;
		float V6;
		float V7;
		float V8;
		float V9;
		float batt_temp;
		int hb;
	}packet_t;

void main() {

	//init variables************************************************************
	//unsigned int streamenabled; 
	//boolean to determine weather to stream to CHREC 
	//RS422 communication unused for flatsat prototype
	
	//solar panels
	unsigned int solar0_iv;
	int16_t solar0_i;
	int16_t solar0_v;
	unsigned int solar1_iv;
	int16_t solar1_i;
	int16_t solar1_v;
	unsigned int solar2_iv;
	int16_t solar2_i;
	int16_t solar2_v;
	unsigned int solar3_iv;
	int16_t solar3_i;
	int16_t solar3_v;


	//bcr output
	unsigned int bcr_iv;
	int16_t bcr_i;
	int16_t bcr_v;

	//battery output
	unsigned int batt_iv;
	int16_t batt_i;
	int16_t batt_v;
	//Power rail outputs
	unsigned int rail33_iv;
	int16_t rail33_i;
	int16_t rail33_v;
	unsigned int rail5_iv;
	int16_t rail5_i;
	int16_t rail5_v;
	unsigned int rail12_iv;
	int16_t rail12_i;
	int16_t rail12_v;
	unsigned int rail28_iv;
	int16_t rail28_i;
	int16_t rail28_v;
	unsigned int batt_temp;

	//PDM Consectutive Fault Counter
	unsigned int PDM0_fault_count = 0;
	unsigned int PDM1_fault_count = 0;
	unsigned int PDM2_fault_count = 0;
	unsigned int PDM3_fault_count = 0;
	unsigned int PDM4_fault_count = 0;
	unsigned int PDM5_fault_count = 0;
	unsigned int PDM6_fault_count = 0;
	unsigned int PDM7_fault_count = 0;
	unsigned int PDM8_fault_count = 0;
	unsigned int PDM9_fault_count = 0;
	unsigned int PDM10_fault_count = 0;
	unsigned int PDM11_fault_count = 0;
	
	//variables needed for UDP
	struct sockaddr_in si_other;
	int s, i, slen=sizeof(si_other);
	packet_t EPS_packet;
	char stream[sizeof(EPS_packet)];
	
	//Setup*********************************************************************
	
	//Temp Setup
	if (initialize_mmap_adc() != 0) {	
		printf("adc_init_failure\n");
	}
	//UDP Setup
	snprintf( stream,sizeof(EPS_packet), "%c", &EPS_packet);
	
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {	
		printf("something went wrong idk\n");
		exit(1);
	}
        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(PORT);
        if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
			printf("somethin failed brah\n");
			exit(1);
		}
	
	//Export GPIO Pins on Beaglebone
	gpio_export(PCM_IN_EN);
	gpio_export(BCR_OUT_EN);
	//gpio_export(BCR0_EN);
	//gpio_export(BCR1_EN);
	//gpio_export(BCR2_EN);
	//gpio_export(BCR3_EN);
	gpio_export(PDM0_EN);
	gpio_export(PDM1_EN);
	gpio_export(PDM2_EN);
	gpio_export(PDM3_EN);
	gpio_export(PDM4_EN);
	gpio_export(PDM5_EN);
	//gpio_export(PDM6_EN);
	//gpio_export(PDM7_EN);
	//gpio_export(PDM8_EN);
	//gpio_export(PDM9_EN);
	//gpio_export(PDM10_EN);
	//gpio_export(PDM11_EN);
	//gpio_export(HEATER_EN);
	gpio_export(PDM0_FAULT);
	gpio_export(PDM1_FAULT);
	gpio_export(PDM2_FAULT);
	gpio_export(PDM3_FAULT);
	gpio_export(PDM4_FAULT);
	gpio_export(PDM5_FAULT);
	gpio_export(PDM6_FAULT);
	//gpio_export(PDM7_FAULT);
	//gpio_export(PDM8_FAULT);
	//gpio_export(PDM9_FAULT);
	//gpio_export(PDM10_FAULT);
	//gpio_export(PDM11_FAULT);
	//gpio_export(TESTMODE_PIN); //Testmode unused for flatsat
	
	//Intialize all pins to low
	gpio_set_value(PCM_IN_EN,LOW);
	gpio_set_value(BCR_OUT_EN,LOW);
	//gpio_set_value(BCR0_EN,LOW);
	//gpio_set_value(BCR1_EN,LOW);
	//gpio_set_value(BCR2_EN,LOW);
	//gpio_set_value(BCR3_EN,LOW);
	gpio_set_value(PDM0_EN,LOW);
	gpio_set_value(PDM1_EN,LOW);
	gpio_set_value(PDM2_EN,LOW);
	gpio_set_value(PDM3_EN,LOW);
	gpio_set_value(PDM4_EN,LOW);
	gpio_set_value(PDM5_EN,LOW);
	// gpio_set_value(PDM6_EN,LOW);
	// gpio_set_value(PDM7_EN,LOW);
	// gpio_set_value(PDM8_EN,LOW);
	// gpio_set_value(PDM9_EN,LOW);
	// gpio_set_value(PDM10_EN,LOW);
	// gpio_set_value(PDM11_EN,LOW);
	// gpio_set_value(HEATER_EN,LOW);
	
	//set gpio pin directions
	gpio_set_dir(PCM_IN_EN, 1);
	gpio_set_dir(BCR_OUT_EN, 1);
	// gpio_set_dir(BCR0_EN, 0);
	// gpio_set_dir(BCR1_EN, 0);
	// gpio_set_dir(BCR2_EN, 0);
	// gpio_set_dir(BCR3_EN, 0);
	gpio_set_dir(PDM0_EN, 1);
	gpio_set_dir(PDM1_EN, 1);
	gpio_set_dir(PDM2_EN, 1);
	gpio_set_dir(PDM3_EN, 1);
	gpio_set_dir(PDM4_EN, 1);
	gpio_set_dir(PDM5_EN, 1);
	// gpio_set_dir(PDM6_EN, 1);
	// gpio_set_dir(PDM7_EN, 1);
	// gpio_set_dir(PDM8_EN, 1);
	// gpio_set_dir(PDM9_EN, 1);
	// gpio_set_dir(PDM10_EN, 1);
	// gpio_set_dir(PDM11_EN, 1);
	// gpio_set_dir(HEATER_EN, 1);
	gpio_set_dir(PDM0_FAULT, 0);
	gpio_set_dir(PDM1_FAULT, 0);
	gpio_set_dir(PDM2_FAULT, 0);
	gpio_set_dir(PDM3_FAULT, 0);
	gpio_set_dir(PDM4_FAULT, 0);
	gpio_set_dir(PDM5_FAULT, 0);
	gpio_set_dir(PDM6_FAULT, 0);
	// gpio_set_dir(PDM7_FAULT, 0);
	// gpio_set_dir(PDM8_FAULT, 0);
	// gpio_set_dir(PDM9_FAULT, 0);
	// gpio_set_dir(PDM10_FAULT, 0);
	// gpio_set_dir(PDM11_FAULT, 0);
	
	
	//initialize the network packet
	EPS_packet.PDM0 = 0;
	EPS_packet.PDM1 = 0;
	EPS_packet.PDM2 = 0;
	EPS_packet.PDM3 = 0;
	EPS_packet.PDM4 = 0;
	EPS_packet.PDM5 = 0;
	EPS_packet.PDM6 = 0;
	EPS_packet.PDM7 = 0;
	EPS_packet.PDM8 = 0;
	EPS_packet.PDM9 = 0;
	EPS_packet.PDM10 = 0;
	EPS_packet.PDM11 = 0;
	EPS_packet.I0 = 0.00;
	EPS_packet.I1 = 0.00;
	EPS_packet.I2 = 0.00;
	EPS_packet.I3 = 0.00;
	EPS_packet.I4 = 0.00;
	EPS_packet.I5 = 0.00;
	EPS_packet.I6 = 0.00;
	EPS_packet.I7 = 0.00;
	EPS_packet.I8 = 0.00;
	EPS_packet.I9 = 0.00;
	EPS_packet.V0 = 0.00;
	EPS_packet.V1 = 0.00;
	EPS_packet.V2 = 0.00;
	EPS_packet.V3 = 0.00;
	EPS_packet.V4 = 0.00;
	EPS_packet.V5 = 0.00;
	EPS_packet.V6 = 0.00;
	EPS_packet.V7 = 0.00;
	EPS_packet.V8 = 0.00;
	EPS_packet.V9 = 0.00;
	EPS_packet.batt_temp = 0.00;
	EPS_packet.hb = 0;

	
	//measure battery voltage until it comes into an acceptable range	
	/* do 
	{
		config_INA3221(0);
		usleep(CONVERSION_TIME_INA3221);
		batt_iv = get_iv_INA3221(0,2);
		batt_i = (solar0_iv >> 16);
		batt_v = solar0_iv;
		EPS_packet.I3 = (double)solar0_i*0.04/(8*2) + INA3221_OFFSET;
		EPS_packet.V3 = (double)solar0_v*0.008/8;
		printf("\r i = %lf, ",EPS_packet.I3);
		printf(" v = %lf",EPS_packet.V3);
	} while(EPS_packet.V3 < 6.0 || EPS_packet.V3 > 9.0);  */
	
	//wait 15 seconds for power up
	printf("\n\n\n\n\n\n\n\n\n");
	int timer;
	for(timer = 1; timer > 0; timer--) {
		printf("Power on in %d seconds...\n", timer);
		sleep(1);
	}	
	
	
	
	gpio_set_value(PCM_IN_EN,1);
	gpio_set_value(BCR_OUT_EN,1);
	gpio_set_value(PDM0_EN,1);
	gpio_set_value(PDM1_EN,1);
	gpio_set_value(PDM2_EN,1);
	gpio_set_value(PDM3_EN,1);
	gpio_set_value(PDM4_EN,1);
	gpio_set_value(PDM5_EN,1);
	
	
    int heartbeat = 0;
	printf("|   BCR OUT   |     BATT    |    3.3V     |     5V      |     12V     |     28V     | TEMP |\n");
	while(1) {
		//heartbeat indicator
		heartbeat = heartbeat + 1;
		EPS_packet.hb = heartbeat%2;


		//get PDM Fault Values
		gpio_get_value(PDM0_FAULT,&EPS_packet.PDM0);
		gpio_get_value(PDM1_FAULT,&EPS_packet.PDM1);
		gpio_get_value(PDM2_FAULT,&EPS_packet.PDM2);
		gpio_get_value(PDM3_FAULT,&EPS_packet.PDM3);
		gpio_get_value(PDM4_FAULT,&EPS_packet.PDM4);
		gpio_get_value(PDM5_FAULT,&EPS_packet.PDM5);
		gpio_get_value(PDM6_FAULT,&EPS_packet.PDM6);
		
		//TEMP
		EPS_packet.batt_temp = (float)mmap_adc_read_raw(0)*1.8*1000000/4095/994 - 273.2 + \
		TEMP_CALIBRATION;
		
		//trigger a measurement on the IV monitors
		config_INA3221(0);
		config_INA3221(1);
		config_INA3221(2);
		config_INA260(0);
		//wait for conversion
		usleep(CONVERSION_TIME_INA3221);
		
		solar3_iv = get_iv_INA3221(0,0);
		solar3_i = (solar3_iv >> 16);
		solar3_v = solar3_iv;
		bcr_iv = get_iv_INA3221(0,1);
		bcr_i = (bcr_iv >> 16);
		bcr_v = bcr_iv;
		batt_iv = get_iv_INA3221(0,2);
		batt_i = (batt_iv >> 16);
		batt_v = batt_iv;
		rail33_iv = get_iv_INA3221(1,0);
		rail33_i = (rail33_iv >> 16);
		rail33_v = rail33_iv;
		rail5_iv = get_iv_INA3221(1,1);
		rail5_i = (rail5_iv >> 16);
		rail5_v = rail5_iv;
		rail12_iv = get_iv_INA3221(1,2);
		rail12_i = (rail12_iv >> 16);
		rail12_v = rail12_iv;
		solar0_iv = get_iv_INA3221(2,0);
		solar0_i = (solar0_iv >> 16);
		solar0_v = solar0_iv;
		solar1_iv = get_iv_INA3221(2,1);
		solar1_i = (solar1_iv >> 16);
		solar1_v = solar1_iv;
		solar2_iv = get_iv_INA3221(2,2);
		solar2_i = (solar2_iv >> 16);
		solar2_v = solar2_iv;
		rail28_iv = get_iv_INA260(0);
		rail28_i = (rail28_iv >> 16);
		rail28_v = rail28_iv;
		
		EPS_packet.I0 = (double)solar3_i*0.04/(8*2);
		EPS_packet.V0 = (double)solar3_v*0.008/8;
		
		EPS_packet.I1 = (double)bcr_i*0.04/(8*2);
		EPS_packet.V1 = (double)bcr_v*0.008/8;
		printf("\r|%5.2fA|",EPS_packet.I1);
		printf("%5.2f|V",EPS_packet.V1);
		
		EPS_packet.I2 = (double)batt_i*0.04/(8*2);
		EPS_packet.V2 = (double)batt_v*0.008/8;
		printf("%5.2fA|",EPS_packet.I2);
		printf("%5.2fV|",EPS_packet.V2);
		
		EPS_packet.I3 = (double)rail33_i*0.04/(8*2);
		EPS_packet.V3 = (double)rail33_v*0.008/8;
		printf("%5.2fA|",EPS_packet.I3);
		printf("%5.2fV|",EPS_packet.V3);
		
		EPS_packet.I4 = (double)rail5_i*0.04/(8*2);
		EPS_packet.V4 = (double)rail5_v*0.008/8;
		printf("%5.2fA|",EPS_packet.I4);
		printf("%5.2fV|",EPS_packet.V4);
		
		EPS_packet.I5 = (double)rail12_i*0.04/(8*2);
		EPS_packet.V5 = (double)rail12_v*0.008/8;
		printf("%5.2fA|",EPS_packet.I5);
		printf("%5.2fV|",EPS_packet.V5);
		
		EPS_packet.I6 = (double)solar0_i*0.04/(8*2);
		EPS_packet.V6 = (double)solar0_v*0.008/8;
		
		EPS_packet.I7 = (double)solar1_i*0.04/(8*2);
		EPS_packet.V7 = (double)solar1_v*0.008/8;

		EPS_packet.I8 = (double)solar2_i*0.04/(8*2);
		EPS_packet.V8 = (double)solar2_v*0.008/8;
		
		EPS_packet.I9 = (double)rail28_i*0.00125;
		EPS_packet.V9 = (double)rail28_v*0.00125;
		printf("%5.2fA|",EPS_packet.I9);
		printf("%5.2fV|",EPS_packet.V9);
		
		printf("%5.2fC|", EPS_packet.batt_temp);
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM0 == 1) {
			if (PDM0_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM0_EN,0);
				usleep(1000);
				gpio_set_value(PDM0_EN,1);
				PDM0_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM0_EN,0);
			
		}
		else PDM0_fault_count = 0;
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM1 == 1) {
			if (PDM1_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM1_EN,0);
				usleep(1000);
				gpio_set_value(PDM1_EN,1);
				PDM1_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM1_EN,0);
			
		}
		else PDM1_fault_count = 0;
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM2 == 1) {
			if (PDM2_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM2_EN,0);
				usleep(1000);
				gpio_set_value(PDM2_EN,1);
				PDM2_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM2_EN,0);
			
		}
		else PDM2_fault_count = 0;
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM3 == 1) {
			if (PDM3_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM3_EN,0);
				usleep(1000);
				gpio_set_value(PDM3_EN,1);
				PDM3_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM3_EN,0);
			
		}
		else PDM3_fault_count = 0;
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM4 == 1) {
			if (PDM4_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM4_EN,0);
				usleep(1000);
				gpio_set_value(PDM4_EN,1);
				PDM4_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM4_EN,0);
			
		}
		else PDM4_fault_count = 0;
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM5 == 1) {
			if (PDM5_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM5_EN,0);
				usleep(1000);
				gpio_set_value(PDM5_EN,1);
				PDM5_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM5_EN,0);
			
		}
		else PDM5_fault_count = 0;
		
		//unlatch PDM if the fault_count pin is high
		if (EPS_packet.PDM6 == 1) {
			if (PDM6_fault_count < 3) {
				//unlatch PDM
				gpio_set_value(PDM5_EN,0);
				usleep(1000);
				gpio_set_value(PDM5_EN,1);
				PDM1_fault_count++;
			}
			//permanently disable PDM if 3 attempts are made
			else gpio_set_value(PDM5_EN,0);
			
		}
		else PDM6_fault_count = 0;
		
		if (sendto(s, &EPS_packet, sizeof(EPS_packet), 0, (struct sockaddr *)&si_other, slen)==-1) {
			printf("send error\n");
			exit(1);
		}
		//stream debug
		/* for (i=0; i<sizeof(EPS_packet); i++) {
			printf("%x",stream[i]);
		}  */

		usleep(1000000);
	}
	
	close(s);
	return;
}
