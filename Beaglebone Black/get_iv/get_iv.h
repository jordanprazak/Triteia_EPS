#ifndef GET_IV_H_
#define GET_IV_H_
/*
 *
 *get_iv.h
 *
 *Contains 2 functions that use the i2c bus to communicate with the 2 iv monitoring chips on the EPS.
 */

#define SHUNT_VALUE .002
#define INA260_CURRENT_LSB 0.00125
#define INA260_VOLTAGE_LSB 0.00125
#define INA3221_CURRENT_LSB 0.00004
#define INA3221_VOLTAGE_LSB 0.008
#define ADDRESS_LIST_INA260 0x43

//config defines
//bit15 = Reset (Keep Low)
//bit14-12 = Channel Enable (Keep High)
//bit11-9 = channel average count
//bit8-6 = bus voltage conversion time
//bit5-3 = shunt voltage conversion time
//bit2-0 = operating mode
#define INA3221_CONFIG_MSB 0b01110011
#define INA3221_CONFIG_LSB 0b00111011

//config defines
//bit15 = Reset (Keep Low)
//bit14-12 = Unused
//bit11-9 = channel average count
//bit8-6 = bus voltage conversion time
//bit5-3 = shunt voltage conversion time
//bit2-0 = operating mode
#define INA260_CONFIG_MSB 0b01100011
#define INA260_CONFIG_LSB 0b00100011
        
#define ADDRESS_LIST_INA3221 \
        0x40,\
        0x41,\
        0x42


  
int config_INA260(int chip_select);
unsigned int get_iv_INA260(int chip_select);

int config_INA3221(int chip_select);
unsigned int get_iv_INA3221(int chip_select, int channel_select);
#endif