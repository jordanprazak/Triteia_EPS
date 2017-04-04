/*      C code to implement EPS protection Schemes        */
#include <stdio.h>
#include <stdint.h>
#include "EPS.h"
#include "EPS_proto.h"

int main() {

  // Boolean to determine whether to send data (stream) to CHREC processor
  unsigned int streamenabled;

  // 32 bit iv values: high 16 will be current, low 16 will be voltage (TODO might be flipped)
  // Solar panel output
  struct iv_int solar0_iv, solar1_iv, solar2_iv, solar3_iv;

  // BCR output
  struct iv_int bcr_iv;

  // Battery output
  struct iv_int batt_iv;

  // Power rail outputs
  struct iv_int rail33_iv, rail5_iv, rail12_iv, rail28_iv;

  // Battery temperature
  unsigned int batt_temp;

  // GPIO pins
  int BCR_EN[] = { BCR0_EN, BCR1_EN, BCR2_EN, BCR3_EN };
  int PDM_EN[] = { PDM0_EN, PDM1_EN, PDM2_EN, PDM3_EN, PDM4_EN, PDM5_EN,
                    PDM6_EN, PDM7_EN, PDM8_EN, PDM9_EN, PDM10_EN, PDM11_EN,
                    PDM12_EN };

  // Export GPIO pins for use
  gpio_export( EPS_OUT_EN );
  gpio_export( BCR_OUT_EN );
  for (int i : BCR_EN) {
    gpio_export( i );
  }
  for (int i : PDM_EN) {
    gpio_export( i );
  }
  gpio_export( HEATER_EN );
  gpio_export( TESTMODE_PIN );

  // Set gpio pin directions
  gpio_set_dir( EPS_OUT_EN, 1 );

  /****************************************************************************
   *                          EPS Power-on sequence                           *
   ****************************************************************************/

  //The cubesat has been powered on into flight mode
  if( gpio_get_value( TESTMODE_PIN ) == 1 )
  {

    // TODO First, we need to make sure EVERYTHING is OFF (latched, disconnect).

  	// Measure battery voltage until it comes into an acceptable range
  	do {
  		batt_iv.iv = get_iv_INA3221( 0, 0 );
  		batt_iv.v = (int16_t) (batt_iv.iv >> 16);
  	} while( batt_iv.v < DEPLOYED_BATT_LOW || batt_iv.v > DEPLOYED_BATT_HIGH );

    // Next, check to see if there is voltage and current coming from one of the
    // solar panels. If so, the cubesat is deployed.
  	do {
  		solar0_iv.iv = get_iv_INA3221( 0, 0 );
  		solar0_iv.i = (int16_t) solar0_iv.iv;
  		solar0_iv.v = (int16_t) (solar0_iv.iv >> 16);

      solar1_iv.iv = get_iv_INA3221( 0, 0 );
  		solar1_iv.i = (int16_t) solar1_iv.iv;
  		solar1_iv.v = (int16_t) (solar1_iv.iv >> 16);

      solar2_iv.iv = get_iv_INA3221( 0, 0 );
  		solar2_iv.i = (int16_t) solar2_iv.iv;
  		solar2_iv.v = (int16_t) (solar2_iv.iv >> 16);

      solar3_iv.iv = get_iv_INA3221( 0, 0 );
  		solar3_iv.i = (int16_t) solar3_iv.iv;
  		solar3_iv.v = (int16_t) (solar3_iv.iv >> 16);

  	} while( ( solar0_iv.v > DEPLOYED_SOLAR_V && solar0_iv.i > DEPLOYED_SOLAR_I ) ||
          	( solar1_iv.v > DEPLOYED_SOLAR_V && solar1_iv.i > DEPLOYED_SOLAR_I ) ||
          	( solar2_iv.v > DEPLOYED_SOLAR_V && solar2_iv.i > DEPLOYED_SOLAR_I ) ||
          	( solar3_iv.v > DEPLOYED_SOLAR_V && solar3_iv.i > DEPLOYED_SOLAR_I )
           );

  	// Cubesat has been successfully deployed. Wait at least 15 seconds, and
    // then power on all the loads.
  	usleep( 1500000 );

  	gpio_set_value( EPS_OUT_EN, HIGH ); //enable EPS output
    for (int i : PDM_EN) {
      gpio_set_value( i, HIGH );	//enable load 0
    }
  }

  // Cubesat has been powered on into test mode
  else {

    // Measure battery voltage until it comes into an acceptable range
  	do {
  		batt_iv.iv = get_iv_INA3221( 0, 0 );
  		batt_iv.v = (int16_t) (batt_iv.iv >> 16);
  	} while( batt_iv.v > TESTMODE_BATT );

  	// Disable BCR outputs
    for (int i : BCR_EN) {
      gpio_set_value( i, 0 );
    	gpio_set_dir( i, 1 );
    }

  	// Power on the CHREC processor and any other necessary loads.
  	gpio_set_value( EPS_OUT_EN, HIGH );	// Enable EPS output
  	gpio_set_value( PDM0_EN, HIGH );		// Enable load 0
  }

  /*Main execution loop goes here
   *sample all of the iv channels
   *sample_iv will be a function that takes in which chip to monitor and
   *returns an unsigned int that contains a voltage, current, and power
   *measurement*/
  while( 1 ) {

    /**************************************************************************
     *             Voltage, Current, and Temperature Monitoring               *
     **************************************************************************/

    solar0_iv.iv = get_iv_INA3221( 0, 0 );
  	solar1_iv.iv = get_iv_INA3221( 0, 1 );
  	solar2_iv.iv = get_iv_INA3221( 0, 2 );
  	solar3_iv.iv = get_iv_INA3221( 1, 0 );
  	bcr_iv.iv = get_iv_INA3221( 1, 1 );
    batt_iv.iv = get_iv_INA3221( 1, 2 );
    rail33_iv.iv = get_iv_INA3221( 2, 0 );
    rail5_iv.iv = get_iv_INA3221( 2, 1 );
    rail12_iv.iv = get_iv_INA3221( 2, 2 );
    rail28_iv.iv = get_iv_INA260( 0 );

    batt_temp = mmap_adc_read_raw( 0 );

    /**************************************************************************
     *                       Battery: Over-voltage Check                      *
     **************************************************************************/

    // If battery voltage is too high, open BCR switch
    if( batt_iv.v > BATT_OVER_VOLTAGE ) {
      gpio_set_value( BCR_OUT_EN, 0 );
    }
    else {

      // TODO we need to figure out a turn-on BCR voltage after over voltage
      gpio_set_value( BCR_OUT_EN, 1 );
    }

    /**************************************************************************
     *                       Battery: Under-voltage Check                     *
     **************************************************************************/

     // If battery voltage is too low, open PCM switch
      if( batt_iv.v < BATT_UNDER_VOLTAGE ) {
        for (int i : PDM_EN) {
          gpio_set_value( i, 0 );
        }
      }
      else {

        // TODO we need to figure out a turn-on PCM after under voltage
        for (int i : PDM_EN) {
          gpio_set_value( i, 1 );
        }
      }

    /**************************************************************************
     *                     Power Rails: Over-current Check                    *
     **************************************************************************/

     // TODO
     // So technically, we have control of every payload.
     // Since the re-latching time is very quick, it will have no effect with
     // the 1 Hz loop code checking 3 times (3 seconds). We will need an
     // internal static counter. If after 3 seconds it's still latched, we have
     // a permanent fault.

      // if there is an over current on any rails, disable the rail, if not, keep them on
      // if (rail33_i > 33RAIL_OVER_CURRENT)
          // set_pdm_en( 0, 0 );
      // else
          // set_pdm_en( 0, 1 );

      // if (rail5_i > 5RAIL_OVER_CURRENT)
          // set_pdm_en( 1, 0 );
      // else
          // set_pdm_en( 1, 1 );

      // if (rail12_i > 12RAIL_OVER_CURRENT)
          // set_pdm_en( 2, 0 );
      // else
          // set_pdm_en( 2, 1 );

      // if (rail28_i > 28RAIL_OVER_CURRENT)
          // set_pdm_en( 3, 0 );
      // else
          // set_pdm_en( 3, 1 );

    /**************************************************************************
     *                       Battery Temperature Check                        *
     **************************************************************************/

      // If the battery temperature is too low, turn on heater until the
      // desired temperature is reached.
      if( batt_temp < HEATER_ON_TEMP ) {
        gpio_set_value( HEATER_EN, 1 );
      }
      else if( batt_temp > HEATER_OFF_TEMP ) {
        gpio_set_value( HEATER_EN, 0 );
      }

    /**************************************************************************
     *                     Sending Data to CHREC processor                    *
     **************************************************************************/
  	//send data is unused for the subsystem prototype
      // if the stream is enabled, send data to CHREK processor
      //this process may run on a separate timer instead
      //if (streamenabled==1)
          //send_data();

    // Sample at a rate of 1 Hz
  	usleep(1000000);
  }

  return EXIT_SUCCESS;
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
