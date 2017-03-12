#ifndef GET_IV_H_
#define GET_IV_H_
/*
 *
 *get_iv.h
 *
 *Contains 2 functions that use the i2c bus to communicate with the 2 iv monitoring chips on the EPS.
 */

#define SHUNT_VALUE 5
#define INA260_CURRENT_LSB 0.00125
#define INA260_VOLTAGE_LSB 0.00125
#define INA3221_CURRENT_LSB 0.00008
#define INA3221_VOLTAGE_LSB 0.008
#define ADDRESS_LIST_INA260 \
        0b00000000,\
        0b00000001,\
        0b00000010,\
        0b00000011,\
        0b00000100,\
        0b00000101,\
        0b00000110,\
        0b00000111,\
        0b00001000,\
        0b00001001,\
        0b00001010
        
#define ADDRESS_LIST_INA3221 \
        0b00000000,\
        0b00000001,\
        0b00000010,\
        0b00000011,\
        0b00000100,\
        0b00000101,\
        0b00000110,\
        0b00000111,\
        0b00001000,\
        0b00001001,\
        0b00001010


typedef enum
{
    AMPS,
    VOLTS,
}
iv_mode_t;        

unsigned int get_iv_INA260(int chip_select);
unsigned int get_iv_INA3221(int chip_select, int channel_select);
#endif