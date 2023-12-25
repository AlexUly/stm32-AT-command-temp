/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// define global variable
input_buf_t project_input_buf;
int msgLen = 0;
char commandBuf[INPUT_BUF_SIZE + 1];
uint16_t readADC = 0;
int MODE = 0;
const int MODE_COUNT = 3;
int BLINK_COUNT = 0;
uint16_t LIGHTS_MASK = 0;
char valADC[16];
int TIMESTAMP = 0;
int OVERFLOW_LEVEL = 3000;
/**
 * Initialize input_line_buf_t object.
 */
int input_buf_init(input_buf_t *input_buf, UART_HandleTypeDef *huart) {
	 memset((void*) input_buf->buf, 0, INPUT_BUF_SIZE);
	 input_buf->pos = 0;
	 input_buf->huart = huart;
	 // enable interrupt
	 __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
	 return 0;
}
/**
 * Read current content of the buffer.
 */
int input_buf_read_content(input_buf_t *input_buf, char *output) {
	 int pos = input_buf->pos;
		 // copy data to `output` buffer
		 for (int i = 0; i < INPUT_BUF_SIZE; i++) {
			 output[i] = input_buf->buf[pos];
			 pos++;
			 if (pos >= INPUT_BUF_SIZE) {
				 pos = 0;
			 }
	 }
	 return INPUT_BUF_SIZE;
}


/**
 * Process UART interruption.
 */
int input_buf_process_rxne_it(input_buf_t *input_buf) {
	 // ignore interrupt if it isn't related with received data
	 if (!__HAL_UART_GET_FLAG(input_buf->huart, UART_FLAG_RXNE)) {
		 return 0;
	 }
	 // process received data
	 char sym = input_buf->huart->Instance->RDR;
	 // save symbol into buffer
	 size_t pos = input_buf->pos;
	 input_buf->buf[pos] = sym;
	 commandBuf[msgLen % INPUT_BUF_SIZE] = sym;
	 pos++;
	 if (pos >= INPUT_BUF_SIZE) {
		 pos = 0;
	 }
	 msgLen++;
	 input_buf->pos = pos;
	 return 0;
}

void blink_lights(){
	HAL_GPIO_TogglePin(GPIOE, 0xFF00);
	BLINK_COUNT--;
	if(!BLINK_COUNT){
		HAL_GPIO_WritePin(GPIOE, 0xFF00, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LIGHTS_MASK, GPIO_PIN_SET);
	}
}

void read_disp_adc(ADC_HandleTypeDef channel){
	HAL_ADC_Start(&channel);
	HAL_ADC_PollForConversion(&channel, 20);
	readADC = HAL_ADC_GetValue(&channel);
	sprintf(valADC, "\nADC: %d", readADC);
	HAL_UART_Transmit(&huart4, (uint8_t*) valADC, strlen(valADC), HAL_MAX_DELAY);
}

void trig_on_ovwf(ADC_HandleTypeDef channel){
	HAL_ADC_Start(&channel);
	HAL_ADC_PollForConversion(&channel, 20);
	readADC = HAL_ADC_GetValue(&channel);
	if(readADC > OVERFLOW_LEVEL){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
		HAL_UART_Transmit(&huart4, "\nTRIG", 5, HAL_MAX_DELAY);
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
   input_buf_init(&project_input_buf, &huart4);
   char outputBuf[32];
   commandBuf[INPUT_BUF_SIZE] = '\n';

   for(int i = 0; i < INPUT_BUF_SIZE; i++){
   	   commandBuf[i] = '\0';
   }

   char output_buf[INPUT_BUF_SIZE + 32];
   int size;
   int offset;
   char sym;
   MODE = 0;
   HAL_UART_Transmit(&huart4, "Start", 5, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1) {

	   if(BLINK_COUNT)
		   blink_lights();

	   switch(MODE){
	   case 0:
		   break;
	   case 1:
		   if(HAL_GetTick() - TIMESTAMP > 1000){
			   TIMESTAMP = HAL_GetTick();
			   read_disp_adc(hadc1);
		   }
		   break;
	   case 2:
		   trig_on_ovwf(hadc1);
		   break;
	   default:
		   break;
	   }


	   strcpy(output_buf, "input buf: \"");
	   offset = strlen(output_buf);
	   if(msgLen){
		   int n = 0;
		   int cmd_len = strlen(commandBuf);
		   sscanf(commandBuf, "AT+SAYHELLO%n", &n);

		   if (n == cmd_len & !strncmp(commandBuf, "AT+SAYHELLO", 11)){
			   strcpy(outputBuf, "Hello world!");
		   }

		   sscanf(commandBuf, "AT+TOGGLELIGHTS%n", &n);
		   if (n == cmd_len & !strncmp(commandBuf, "AT+TOGGLELIGHTS", 15)){
			   if(!BLINK_COUNT)
				   HAL_GPIO_TogglePin(GPIOE, 0xFF00);
		   }

		   sscanf(commandBuf, "AT+READCHANNEL=%*d,%n", &n);
		   if (!strncmp(commandBuf, "AT+READCHANNEL=", 15)){
			   int chnl = 0;
			   sscanf(commandBuf, "AT+READCHANNEL=%d", &chnl);
			   ADC_HandleTypeDef channel;
			   int flag = 0;
			   switch(chnl){
			   case 1:
				   channel = hadc1;
				   flag = 1;
				   break;
			   default:
				   break;
			   }
			   if(flag){
				   read_disp_adc(channel);
			   	   strcpy(outputBuf, "OK");
			   }
			   else
				   strcpy(outputBuf, "ERROR");
		   }

		   sscanf(commandBuf, "AT+MASKLIGHTS=%*d,%n", &n);
		    if (!strncmp(commandBuf, "AT+MASKLIGHTS=", 14)){
		    	HAL_UART_Transmit(&huart4, "IN\n", 3, HAL_MAX_DELAY);
		    	uint8_t mask = 0;
		    	sscanf(commandBuf, "AT+MASKLIGHTS=%d", &mask);
		    	if(!BLINK_COUNT){
					HAL_GPIO_WritePin(GPIOE, 0xFF00, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, mask << 8, GPIO_PIN_SET);
		    	}
		   		LIGHTS_MASK = mask << 8;
		    }

			sscanf(commandBuf, "AT+TIME%n", &n);
			if (!strncmp(commandBuf, "AT+TIME", 7)){
				sprintf(outputBuf, "+TIME:%d", HAL_GetTick());
		    }

		   sscanf(commandBuf, "AT+MODE%n", &n);
			if (!strncmp(commandBuf, "AT+MODE", 7)){
				sprintf(outputBuf, "+MODE:%d", MODE);
			}

			sscanf(commandBuf, "AT+SETMODE:%*d,%n", &n);

			if (!strncmp(commandBuf, "AT+SETMODE:", 11)){
				int tempMode = -1;
				sscanf(commandBuf, "AT+SETMODE:%d", &tempMode);
				if(tempMode >= 0 & tempMode <= MODE_COUNT){
					MODE = tempMode % (MODE_COUNT - 1);
					strcpy(outputBuf, "OK");
				}
				else {
					strcpy(outputBuf, "ERROR");
				}
			}
			if (!strncmp(commandBuf, "AT+OWLEVEL:", 12)){
				int level = -1;
				sscanf(commandBuf, "AT+OWLEVEL:%d", &level);
				if(level >= 0){
					OVERFLOW_LEVEL = level;
					strcpy(outputBuf, "OK");
				}
				else {
					strcpy(outputBuf, "ERROR");
				}
			}
			sscanf(commandBuf, "AT+BLINK=%*d,%n", &n);
			if (!strncmp(commandBuf, "AT+BLINK=", 9)){
				int blinks = 0;
				sscanf(commandBuf, "AT+BLINK=%d", &blinks);
				if(blinks > 0){
					BLINK_COUNT += 2 * blinks;
					HAL_GPIO_WritePin(GPIOE, 0xFF00, GPIO_PIN_RESET);
					//blink_lights(blinks);
					strcpy(outputBuf, "OK");
				}
				else
					strcpy(outputBuf, "ERROR");
			}


		   if(strlen(outputBuf)){
			   HAL_UART_Transmit(&huart4, "\n", 1, HAL_MAX_DELAY);
		   	   HAL_UART_Transmit(&huart4, (uint8_t*) outputBuf, strlen(outputBuf), HAL_MAX_DELAY);
		   }


		   //Clear output buf
		   for(int i = 0; i < INPUT_BUF_SIZE; i++){
		      commandBuf[i] = '\0';
		      outputBuf[i] = '\0';
		   }
		   msgLen = 0;
	   }


	   offset += size;
	   strcpy(output_buf + offset, "\"\n");
	   HAL_Delay(100);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
