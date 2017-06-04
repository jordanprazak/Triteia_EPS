/*SEDS TRITEIA POWER SYSTEMS TEAM prototype EPS code
 *C code to implement EPS protection Schemes
 *STM32 version 0.0.1
 *Authored by Jordan Prazak and David TU
 *
 */


/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* HAL Includes --------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/*EPS includes ---------------------------------------------------------------*/
#include "INA260.h"
#include "EPS.h"

/* HAL Private variables -----------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* EPS Private variables -----------------------------------------------------*/


/* HAL Private function prototypes -------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

/* EPS Private function prototypes -------------------------------------------*/


//TODO add correct indexes for INA260 current and voltage monitoring chips
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Turn on LD2 */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);

	/*Initialize System --------------------------------------------------------*/
	//init variables
	// Boolean to determine whether to send data (stream) to CHREC processor
	int i;
	unsigned int streamenabled;
	EPS_status_t EPS_status;
	
	//configure IV monitors
 	sendConfig( &hi2c1,0);
	sendConfig( &hi2c1,1);
	sendConfig( &hi2c1,2);
	sendConfig( &hi2c1,3);
	sendConfig( &hi2c1,4);
	sendConfig( &hi2c1,5);
	sendConfig( &hi2c1,6);
	sendConfig( &hi2c1,7);
	sendConfig( &hi2c1,8);
	sendConfig( &hi2c1,9);
	
	
/*******************************************************************************
*                          EPS Power-on sequence                               *
*******************************************************************************/

	// Measure battery voltage until it comes into an acceptable range
	do {
		sendConfig( &hi2c1,0);
		EPS_status.batt_v = getVoltage(&hi2c1,0);
	} while(EPS_status.batt_v < DEPLOYED_BATT_LOW || EPS_status.batt_v > DEPLOYED_BATT_HIGH );

	// Next, check to see if there is voltage and current coming from one of the
	// solar panels. If so, the cubesat is deployed.
	do {
		sendConfig( &hi2c1,0);
		EPS_status.solar1_i = getCurrent(&hi2c1,0);
		EPS_status.solar1_v = getVoltage(&hi2c1,0);
		
		sendConfig( &hi2c1,0);
		EPS_status.solar2_i = getCurrent(&hi2c1,0);
		EPS_status.solar2_v = getVoltage(&hi2c1,0);
		
		sendConfig( &hi2c1,0);
		EPS_status.solar3_i = getCurrent(&hi2c1,0);
		EPS_status.solar3_v = getVoltage(&hi2c1,0);
		
		sendConfig( &hi2c1,0);
		EPS_status.solar4_i = getCurrent(&hi2c1,0);
		EPS_status.solar4_v = getVoltage(&hi2c1,0);
		
	} while( 
			( EPS_status.solar1_v > DEPLOYED_SOLAR_V && EPS_status.solar1_i > DEPLOYED_SOLAR_I ) ||
			( EPS_status.solar2_v > DEPLOYED_SOLAR_V && EPS_status.solar2_i > DEPLOYED_SOLAR_I ) ||
			( EPS_status.solar3_v > DEPLOYED_SOLAR_V && EPS_status.solar3_i > DEPLOYED_SOLAR_I ) ||
			( EPS_status.solar4_v > DEPLOYED_SOLAR_V && EPS_status.solar4_i > DEPLOYED_SOLAR_I )
			 );

	// Cubesat has been successfully deployed. Wait at least 15 seconds, and
	// then power on all the loads.
	HAL_DELAY(15000);

	HAL_GPIO_WritePin(GPIOC, PCM_IN_EN_Pin, GPIO_PIN_SET); //enable EPS output
	
	

	}

	
/*******************************************************************************
*             Voltage, Current, and Temperature Monitoring                     *
*******************************************************************************/
    
		// Trigger a measurement on the IV monitors
 		config_INA3221( 0 );
 		config_INA3221( 1 );
 		config_INA3221( 2 );
 		config_INA260( 0 );
 		usleep(CONVERSION_TIME_INA3221);

    solar0_iv.iv = get_iv_INA3221( 0, 0 ); // Demo: solar3
  	solar1_iv.iv = get_iv_INA3221( 0, 1 ); // Demo: bcr
  	solar2_iv.iv = get_iv_INA3221( 0, 2 ); // Demo: batt
  	solar3_iv.iv = get_iv_INA3221( 1, 0 ); // Demo: rail3.3
  	bcr_iv.iv = get_iv_INA3221( 1, 1 ); // Demo: rail5
    batt_iv.iv = get_iv_INA3221( 1, 2 ); // Demo: rail12
    rail33_iv.iv = get_iv_INA3221( 2, 0 ); // Demo: solar0
    rail5_iv.iv = get_iv_INA3221( 2, 1 ); // Demo: solar1
    rail12_iv.iv = get_iv_INA3221( 2, 2 ); // Demo: solar2
    rail28_iv.iv = get_iv_INA260( 0 ); // Demo: rail28

    batt_temp = (float) mmap_adc_read_raw( 0 ) * 1.8 * 1000000 / 4095 / 994 - 273.2 + TEMP_CALIBRATION;

    // Current conversion: i * 0.04/(8*2)
    // Voltage conversion: v * 0.008/8
    // 28V rail, current & voltage: i or v * 0.00125

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
        i = 0;
        while( PDM_EN[i] != NULL ) {
          gpio_set_value( PDM_EN[i++], LOW );
        }
      }
      else {

        // TODO we need to figure out a turn-on PCM after under voltage
        i = 0;
        while( PDM_EN[i] != NULL ) {
          gpio_set_value( PDM_EN[i++], HIGH );
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

     i = 0;
     while( PDM_FAULT[i] != NULL ) {
       if( gpio_get_value( PDM_FAULT[i] ) != 0 ) {
         if( faults[i].fault_count < 3 ) {
           // Attempt to unlatch
           gpio_set_value( PDM_EN[i], 0 );
     			 usleep( 1000 );
     			 gpio_set_value( PDM_EN[i], 1 );
           faults[i].fault_count += 1;
         }
         else {
           // PERMANENT FAULT! TODO SEND TO MISSION CONTROL! But only do it once.
           gpio_set_value( PDM_EN[i], 0 );
         }
       }
       else {
         faults[i].fault_count = 0;
       }
       i += 1;
     }

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
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HEATER_EN_Pin|PL12_EN_Pin|PL7_EN_Pin|PL6_EN_Pin 
                          |PCM_IN_EN_Pin|BCR1_EN_Pin|GPIO_PIN_8|BCR3_EN_Pin 
                          |PL2_FLT_Pin|PL3_FLT_Pin|PL4_FLT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BCR_OUT_EN_Pin|PL11_EN_Pin|LD2_Pin|PL9_EN_Pin 
                          |PL8_EN_Pin|BCR4_EN_Pin|TEST_LED_Pin|STATUS_LED_Pin 
                          |PL1_FLT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PL5_EN_Pin|PL4_EN_Pin|PL3_EN_Pin|PL2_EN_Pin 
                          |PL1_EN_Pin|PCM1_EN_Pin|PCM2_EN_Pin|PCM3_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PL13_FLT_Pin */
  GPIO_InitStruct.Pin = PL13_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PL13_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HEATER_EN_Pin PL12_EN_Pin PL7_EN_Pin PL6_EN_Pin 
                           PCM_IN_EN_Pin BCR1_EN_Pin PC8 BCR3_EN_Pin 
                           PL2_FLT_Pin PL3_FLT_Pin PL4_FLT_Pin */
  GPIO_InitStruct.Pin = HEATER_EN_Pin|PL12_EN_Pin|PL7_EN_Pin|PL6_EN_Pin 
                          |PCM_IN_EN_Pin|BCR1_EN_Pin|GPIO_PIN_8|BCR3_EN_Pin 
                          |PL2_FLT_Pin|PL3_FLT_Pin|PL4_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BCR_OUT_EN_Pin PL11_EN_Pin LD2_Pin PL9_EN_Pin 
                           PL8_EN_Pin BCR4_EN_Pin TEST_LED_Pin STATUS_LED_Pin 
                           PL1_FLT_Pin */
  GPIO_InitStruct.Pin = BCR_OUT_EN_Pin|PL11_EN_Pin|LD2_Pin|PL9_EN_Pin 
                          |PL8_EN_Pin|BCR4_EN_Pin|TEST_LED_Pin|STATUS_LED_Pin 
                          |PL1_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PL5_EN_Pin PL4_EN_Pin PL3_EN_Pin PL2_EN_Pin 
                           PL1_EN_Pin PCM1_EN_Pin PCM2_EN_Pin PCM3_EN_Pin */
  GPIO_InitStruct.Pin = PL5_EN_Pin|PL4_EN_Pin|PL3_EN_Pin|PL2_EN_Pin 
                          |PL1_EN_Pin|PCM1_EN_Pin|PCM2_EN_Pin|PCM3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCM4_EN_Pin PL6_FLT_Pin PL7_FLT_Pin PL8_FLT_Pin 
                           PL9_FLT_Pin */
  GPIO_InitStruct.Pin = PCM4_EN_Pin|PL6_FLT_Pin|PL7_FLT_Pin|PL8_FLT_Pin 
                          |PL9_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PL5_FLT_Pin */
  GPIO_InitStruct.Pin = PL5_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PL5_FLT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Called when a read is completed - works on interrupt */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
