#ifndef EPS_H_
#define EPS_H_

#define TEMP_CALIBRATION 7.3
#define INA3221_OFFSET -0.130
#define CONVERSION_TIME_INA3221 8500
#define CONVERSION_TIME_INA260 1100
#define DEPLOYED_BATT_LOW 0
#define DEPLOYED_BATT_HIGH 0
#define DEPLOYED_SOLAR_V 0
#define DEPLOYED_SOLAR_I 0
#define TESTMODE_BATT 0

#define HEATER_OFF_TEMP 50
#define HEATER_ON_TEMP 0
#define BATT_UNDER_VOLTAGE 7
#define BATT_OVER_VOLTAGE 8.4

//GPIO PIN Definitions
#define PCM_IN_EN		70
#define BCR_OUT_EN		71
#define BCR0_EN			00
#define BCR1_EN			00
#define BCR2_EN			00
#define BCR3_EN 		00
#define PDM0_EN			72	//28 valve en
#define PDM1_EN			60	//28trcv en
#define PDM2_EN			67	//12v trcv en
#define PDM3_EN			81	//5V en
#define PDM4_EN			10	//12v rw en
#define PDM5_EN			49	//33 en
#define PDM6_EN 		00
#define PDM7_EN			00
#define PDM8_EN			00
#define PDM9_EN			00
#define PDM10_EN		00
#define PDM11_EN		00
#define HEATER_EN		00
#define TESTMODE_PIN	00
#define PDM0_FAULT		73	//28 valv fault
#define PDM1_FAULT		31	//28trcv fault
#define PDM2_FAULT		66	//12v trcv fault
#define PDM3_FAULT		80	//5V fault
#define PDM4_FAULT 		11	//12v rw fault
#define PDM5_FAULT		48	//33 fault
#define PDM6_FAULT		20	//5v imu fault
#define PDM7_FAULT		00
#define PDM8_FAULT		00
#define PDM9_FAULT		00
#define PDM10_FAULT		00
#define PDM11_FAULT		00



// /*ADC FUNCTIONS
 // *
 // *All functions required for reading ADC values from the beaglebone black
 // *
 // */
 // #define MMAP_OFFSET (0x44C00000)
// #define MMAP_SIZE   (0x481AEFFF-MMAP_OFFSET)




// /* GPIO Memory Registers */
// #define GPIO_REGISTER_SIZE (4)

// #define GPIO0 	(0x44E07000)
// #define GPIO1		(0x4804C000)
// #define GPIO2		(0x481AC000)
// #define GPIO3		(0x481AE000)

// #define GPIO_CLEARDATAOUT (0x190)
// #define GPIO_SETDATAOUT   (0x194)
// #define GPIO_OE			      (0x134)
// #define GPIO_DATAOUT      (0x13C)
// #define GPIO_DATAIN       (0x138)

// /* Analog Digital Converter Memory Registers */
// #define ADC_TSC (0x44E0D000)

// #define ADC_CTRL (ADC_TSC+0x40)
// #define ADC_STEPCONFIG_WRITE_PROTECT_OFF (0x01<<2)
// #define ADC_STEPENABLE (ADC_TSC+0x54)

// #define ADCSTEPCONFIG1 (ADC_TSC+0x64)
// #define ADCSTEPDELAY1  (ADC_TSC+0x68)
// #define ADCSTEPCONFIG2 (ADC_TSC+0x6C)
// #define ADCSTEPDELAY2  (ADC_TSC+0x70)
// #define ADCSTEPCONFIG3 (ADC_TSC+0x74)
// #define ADCSTEPDELAY3  (ADC_TSC+0x78)
// #define ADCSTEPCONFIG4 (ADC_TSC+0x7C)
// #define ADCSTEPDELAY4  (ADC_TSC+0x80)
// #define ADCSTEPCONFIG5 (ADC_TSC+0x84)
// #define ADCSTEPDELAY5  (ADC_TSC+0x88)
// #define ADCSTEPCONFIG6 (ADC_TSC+0x8C)
// #define ADCSTEPDELAY6  (ADC_TSC+0x90)
// #define ADCSTEPCONFIG7 (ADC_TSC+0x94)
// #define ADCSTEPDELAY7  (ADC_TSC+0x98)
// #define ADCSTEPCONFIG8 (ADC_TSC+0x9C)
// #define ADCSTEPDELAY8  (ADC_TSC+0xA0)

// #define ADC_AVG1 (0b000 << 2)
// #define ADC_AVG2 (0b001 << 2)
// #define ADC_AVG4 (0b010 << 2)
// #define ADC_AVG8 (0b011 << 2)
// #define ADC_AVG16 (0b100 << 2)

// #define ADC_SW_ONESHOT 0b00
// #define FIFO0COUNT (ADC_TSC+0xE4)
// #define FIFO_COUNT_MASK 0b01111111

// #define ADC_FIFO0DATA (ADC_TSC+0x100)
// #define ADC_FIFO_MASK (0xFFF)

// #define TRUE 1
// #define FALSE 0


// #define INPUT    ((unsigned char)(1))
// #define OUTPUT   ((unsigned char)(0))
// #define PULLUP   ((unsigned char)(1))
// #define PULLDOWN ((unsigned char)(0))
// #define PULL_DISABLED ((unsigned char)(2))

// /*********************************
// * clock control registers
// *********************************/
// #ifndef CM_PER
	// #define CM_PER 0x44E00000 //base of Clock Module Peripheral control
	// #define CM_PER_PAGE_SIZE 1024 //1kb
// #endif


// #define CM_PER_GPIO1_CLKCTRL 0xAC
// #define CM_PER_GPIO2_CLKCTRL 0xB0
// #define CM_PER_GPIO3_CLKCTRL 0xB4

// /* Clock Module Memory Registers */
// #define CM_WKUP 0x44E00400

// #define MODULEMODE_DISABLED 0x0
// #define MODULEMODE_ENABLE 	0x2

// #define CM_WKUP_ADC_TSC_CLKCTRL 0xBC
 
#define HIGH 1
#define LOW  0 


////GPIO
// int initialize_mmap_gpio();
// int mmap_gpio_write(int pin, int state);
// int mmap_gpio_read(int pin);

////ADC
// int initialize_mmap_adc();
// int mmap_adc_read_raw(int ch);






// /*
 // * SimpleGPIO.h
 // *
 // * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 // * www.eeng.dcu.ie/~molloyd/
 // *
 // * Based on Software by RidgeRun
 // * Copyright (c) 2011, RidgeRun
 // * All rights reserved.
 // *
 // * Redistribution and use in source and binary forms, with or without
 // * modification, are permitted provided that the following conditions are met:
 // * 1. Redistributions of source code must retain the above copyright
 // *    notice, this list of conditions and the following disclaimer.
 // * 2. Redistributions in binary form must reproduce the above copyright
 // *    notice, this list of conditions and the following disclaimer in the
 // *    documentation and/or other materials provided with the distribution.
 // * 3. All advertising materials mentioning features or use of this software
 // *    must display the following acknowledgement:
 // *    This product includes software developed by the RidgeRun.
 // * 4. Neither the name of the RidgeRun nor the
 // *    names of its contributors may be used to endorse or promote products
 // *    derived from this software without specific prior written permission.
 // *
 // * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 // * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 // * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 // * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 // * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 // * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 // * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 // * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 // * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 // * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 // */




 // /****************************************************************
 // * Constants
 // ****************************************************************/

// #define SYSFS_GPIO_DIR "/sys/class/gpio"
// #define MAX_BUF 64
// #define SYSFS_OMAP_MUX_DIR "/sys/kernel/debug/omap_mux/"

// typedef enum {
	// INPUT_PIN,
	// OUTPUT_PIN
// }PIN_DIRECTION;

// typedef enum {
	// LOW,
	// HIGH
// } PIN_VALUE;

// /****************************************************************
 // * gpio_export
 // ****************************************************************/
// int gpio_export(unsigned int gpio);
// int gpio_unexport(unsigned int gpio);
// int gpio_set_dir(int gpio, PIN_DIRECTION out_flag);
// int gpio_set_value(unsigned int gpio, PIN_VALUE value);
// int gpio_get_value(unsigned int gpio, int *value);
// int gpio_set_edge(unsigned int gpio, char *edge);
// int gpio_fd_open(unsigned int gpio);
// int gpio_fd_close(int fd);
// int gpio_omap_mux_setup(const char *omap_pin0_name, const char *mode);

// /* SIMPLEGPIO_H_ */

// /*
 // *
 // *get_iv.h
 // *
 // *Contains 2 functions that use the i2c bus to communicate with the 2 iv monitoring chips on the EPS.
 // */

// #define SHUNT_VALUE 5
// #define INA260_CURRENT_LSB 0.00125
// #define INA260_VOLTAGE_LSB 0.00125
// #define INA3221_CURRENT_LSB 0.00008
// #define INA3221_VOLTAGE_LSB 0.008
// #define ADDRESS_LIST_INA260 \
        // 0b00000000,\
        // 0b00000001,\
        // 0b00000010,\
        // 0b00000011,\
        // 0b00000100,\
        // 0b00000101,\
        // 0b00000110,\
        // 0b00000111,\
        // 0b00001000,\
        // 0b00001001,\
        // 0b00001010
        
// #define ADDRESS_LIST_INA3221 \
        // 0b00000000,\
        // 0b00000001,\
        // 0b00000010,\
        // 0b00000011,\
        // 0b00000100,\
        // 0b00000101,\
        // 0b00000110,\
        // 0b00000111,\
        // 0b00001000,\
        // 0b00001001,\
        // 0b00001010


// typedef enum
// {
    // AMPS,
    // VOLTS,
// }
// iv_mode_t;        

// unsigned int get_iv_INA260(int chip_select);
// unsigned int get_iv_INA3221(int chip_select, int channel_select);
#endif
