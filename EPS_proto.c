
                    /*      C code to implement EPS protection Schemes        */

/******************************************************************************/
#include<stdio.h>
#include<stdint.h>
#include"EPS.h"
void main() 
{
//init variables
unsigned int streamenabled; //boolean to determine wether to stream to CHREC

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

//export GPIO pins for use
gpio_export(EPS_OUT_EN);
gpio_export(BCR_OUT_EN);
gpio_export(BCR0_EN);
gpio_export(BCR1_EN);
gpio_export(BCR2_EN);
gpio_export(BCR3_EN);
gpio_export(PDM0_EN);
gpio_export(PDM1_EN);
gpio_export(PDM2_EN);
gpio_export(PDM3_EN);
gpio_export(PDM4_EN);
gpio_export(PDM5_EN);
gpio_export(PDM6_EN);
gpio_export(PDM7_EN);
gpio_export(PDM8_EN);
gpio_export(PDM9_EN);
gpio_export(PDM10_EN);
gpio_export(PDM11_EN);
gpio_export(PDM12_EN);
gpio_export(HEATER_EN);
gpio_export(TESTMODE_PIN);

//set gpio pin directions
gpio_set_dir(EPS_OUT_EN, 1);

//set interrupt service routine for each latching current limiter
int16_t isr_latching_current_limiter();

//set interrupt service routine for the UART
int16_t isr_UART();

                                                   
/******************************************************************************/
//Power on sequence
/*verify that the cubesat was deployed by checking that there is a positive 
current on the output of the bcr*/
if (gpio_get_value(TESTMODE_PIN) == 1)
{//The cubesat has been powered on into flight mode
	
	//measure battery voltage until it comes into an acceptable range	
	do 
	{
		batt_iv = get_iv_INA3221(0,0);
		batt_v = (int16_t)(batt_iv >> 16);
	} while(batt_v < DEPLOYED_BATT_LOW || batt_v > DEPLOYED_BATT_HIGH); 
	
	/*check solar panel and voltage until a voltage and current are present on 
	at least one*/
	do 
	{
		//check that there is voltage and current coming from one of the 
		//solar panels
		solar0_iv = get_iv_INA3221(0,0);
		solar0_i = (int16_t)solar0_iv;
		solar0_v = (int16_t)(solar0_iv >> 16);
		solar1_iv = get_iv_INA3221(0,0);
		solar1_i = (int16_t)solar1_iv;
		solar1_v = (int16_t)(solar1_iv >> 16);
		solar2_iv = get_iv_INA3221(0,0);
		solar2_i = (int16_t)solar2_iv;
		solar2_v = (int16_t)(solar2_iv >> 16);
		solar3_iv = get_iv_INA3221(0,0);
		solar3_i = (int16_t)solar3_iv;
		solar3_v = (int16_t)(solar3_iv >> 16);
	} while ((solar0_v > DEPLOYED_SOLAR_V && solar0_i > DEPLOYED_SOLAR_I)||\
	(solar1_v > DEPLOYED_SOLAR_V && solar1_i > DEPLOYED_SOLAR_I) ||\
	(solar2_v > DEPLOYED_SOLAR_V && solar2_i > DEPLOYED_SOLAR_I) ||\
	(solar3_v > DEPLOYED_SOLAR_V && solar3_i > DEPLOYED_SOLAR_I));
	
	/*cubesat has been successfully deployed, wait 15 seconds and then power on
	all the loads*/
	usleep(1500000);
	
	gpio_set_value(EPS_OUT_EN,HIGH);	//enable EPS output
	gpio_set_value(PDM0_EN,HIGH);		//enable load 0
	gpio_set_value(PDM1_EN,HIGH);		//enable load 1
	gpio_set_value(PDM2_EN,HIGH);		//enable load 2
	gpio_set_value(PDM3_EN,HIGH);		//enable load 3
	gpio_set_value(PDM4_EN,HIGH);				//enable load 4
	gpio_set_value(PDM5_EN,HIGH);			//enable load 5
	gpio_set_value(PDM6_EN,HIGH);			//enable load 6
	gpio_set_value(PDM7_EN,HIGH);			//enable load 7
	gpio_set_value(PDM8_EN,HIGH);			//enable load 8
	gpio_set_value(PDM9_EN,HIGH);			//enable load 9
	gpio_set_value(PDM10_EN,HIGH);			//enable load 10
	gpio_set_value(PDM11_EN,HIGH);			//enable load 11
	gpio_set_value(PDM12_EN,HIGH);			//enable load 12
	
}
else 
{
	//Cubesat has been powered on into test mode
	
	//measure battery voltage until it comes into an acceptable range	
	do 
	{
		batt_iv = get_iv_INA3221(0,0);
		batt_v = (int16_t)(batt_iv >> 16);
	} while(batt_v > TESTMODE_BATT); 
	//disable BCR outputs
	gpio_set_value(BCR0_EN,0);
	gpio_set_dir(BCR0_EN, 1);
	gpio_set_value(BCR1_EN,0);
	gpio_set_dir(BCR1_EN, 1);
	gpio_set_value(BCR2_EN,0);
	gpio_set_dir(BCR2_EN, 1);
	gpio_set_value(BCR3_EN,0);
	gpio_set_dir(BCR3_EN, 1);
	
	//power on the CHREC processor and any other necessary loads/
	gpio_set_value(EPS_OUT_EN,HIGH);	//enable EPS output
	gpio_set_value(PDM0_EN,HIGH);		//enable load 0
}


/******************************************************************************/

/*Main execution loop goes here
 *sample all of the iv channels
 *sample_iv will be a function that takes in which chip to monitor and *returns an unsigned int that contains a voltage, current, and power *measurement*/
while(1) 
{
    solar0_iv = get_iv_INA3221(0,0);
	solar1_iv = get_iv_INA3221(0,1);
	solar2_iv = get_iv_INA3221(0,2);
	solar3_iv = get_iv_INA3221(1,0);
	bcr_iv = get_iv_INA3221(1,1);
    batt_iv = get_iv_INA3221(1,2);
    rail33_iv = get_iv_INA3221(2,0);
    rail5_iv = get_iv_INA3221(2,1);
    rail12_iv = get_iv_INA3221(2,2);
    rail28_iv = get_iv_INA260(0);
    batt_temp = mmap_adc_read_raw(0);  //sample the battery temperature

/******************************************************************************/

    //check for battery over voltage
    if (batt_v > BATT_OVER_VOLTAGE) 
        //disconnect bcr
        gpio_set_value(BCR_OUT_EN,0);		
    else 
        gpio_set_value(BCR_OUT_EN,1);

/**************************************************************************************************/

    //check for battery under voltage against threshold
    if (batt_v < BATT_UNDER_VOLTAGE) 
    {
        //disable rails
        gpio_set_value( PDM0_EN, 0); 
        gpio_set_value( PDM1_EN, 0); 
		gpio_set_value( PDM2_EN, 0); 
		gpio_set_value( PDM3_EN, 0); 
		gpio_set_value( PDM4_EN, 0); 
		gpio_set_value( PDM5_EN, 0); 
		gpio_set_value( PDM6_EN, 0); 
		gpio_set_value( PDM7_EN, 0); 
		gpio_set_value( PDM8_EN, 0); 
		gpio_set_value( PDM9_EN, 0); 
		gpio_set_value( PDM10_EN, 0); 
		gpio_set_value( PDM11_EN, 0); 
		gpio_set_value( PDM12_EN, 0); 
    }
    else 
    {
        //enable rails
       //disable rails
        gpio_set_value( PDM0_EN, 1); 
        gpio_set_value( PDM1_EN, 1); 
		gpio_set_value( PDM2_EN, 1); 
		gpio_set_value( PDM3_EN, 1); 
		gpio_set_value( PDM4_EN, 1); 
		gpio_set_value( PDM5_EN, 1); 
		gpio_set_value( PDM6_EN, 1); 
		gpio_set_value( PDM7_EN, 1); 
		gpio_set_value( PDM8_EN, 1); 
		gpio_set_value( PDM9_EN, 1); 
		gpio_set_value( PDM10_EN, 1); 
		gpio_set_value( PDM11_EN, 1); 
		gpio_set_value( PDM12_EN, 1); 
    }
/**************************************************************************************************/

    //if there is an over current on any rails, disable the rail, if not, keep them on
    // if (rail33_i > 33RAIL_OVER_CURRENT) 
        // set_pdm_en( 0, 0);
    // else 
        // set_pdm_en( 0 , 1);

    // if (rail5_i > 5RAIL_OVER_CURRENT) 
        // set_pdm_en( 1, 0);
    // else 
        // set_pdm_en( 1 , 1);

    // if (rail12_i > 12RAIL_OVER_CURRENT) 
        // set_pdm_en( 2, 0);
    // else 
        // set_pdm_en( 2 , 1);

    // if (rail28_i > 28RAIL_OVER_CURRENT) 
        // set_pdm_en( 3, 0);
    // else 
        // set_pdm_en( 3 , 1);

/**************************************************************************************************/

    //if the battery temp is low, turn on heater until desired temp is reached
    if (batt_temp < HEATER_ON_TEMP) 
        gpio_set_value(HEATER_EN, 1); 
    else if (batt_temp > HEATER_OFF_TEMP) 
        gpio_set_value(HEATER_EN, 0);

/**************************************************************************************************/
	//send data is unused for the subsystem prototype
    // if the stream is enabled, send data to CHREK processor
    //this process may run on a separate timer instead
    //if (streamenabled==1) 
        //send_data();
	//usleep(1000000);
}

return;
}
/*********************************************************************************************************/
//uart function is not implemented for the flat sat prototype
//int isr_UART() 
//{	
/*interrupt service routine for the UART interrupt
 *This code will operate whenever the UART receives data from the chrec 
 *processor*/
//read_uart_data() 
//read data in from the uart transceiver
//switch(command) 
//{
	//case 10 : 
		//streamenabled = TRUE; //start data streaming
	//case 11 : 
		//streamenabled = FALSE; //stop data streaming
	//more cases can be added in same manner	 
//}
//return 0;
//}

/*************************************************************************  ***/

// int isr_latching_current_limiter()
// {	
// /*if this routine is called, then a latching current limiter has latched, there has been an OC or OV  situation */
//first find current state values for each lcl
// get_lcl_fault_values();

// for (i=0; i<=3; i++) 
// {	
////check all the lcl fault lines
	// if (lcl_fault(i) == 1 && ignore_lcl_fault(i) == 0) 
    // {
	    // fault_count = 0;
	    // while (fault_count <=4) 
        // {	
            //attempt the reset 5 times
			//if a fault line is high, attempt to reset it
			// set_pdm_en( i, 0);
			// usleep(100); 	//leave the signal low for 100 microseconds
			// set_pdm_en( i, 1);
			// usleep(1000); 	//wait 1 millisecond to see if the reset worked

			// if (lcl_fault(i) == 0) 
                // break;	//if the fault cleared, exit the loop
			// fault_count++; 

            ////increment fault count if we failed to unlatch the current limiter
            // if (fault_count == 5) 
            // {
                // set_pdm_fault( i, 1);
                // /*if we fail after 5 tries, set a fault condition to notify the chrek processor*/
                // ignore_lcl_fault(i) = 1; 	
                //ignore the fault in the future since we can’t clear it 

            // }
        // }
	// }
// }
// return 0;
// }

/*********************************************************************************************************************/