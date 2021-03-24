/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* Command Defines -----------------------------------------------------------*/
#define MESSAGE_HEADER  0x01
#define LED_ON			0x01
#define LED_OFF			0x02
#define LED_TOGGLE 		0x03
#define LOOP_BACK		0x04
#define ADC_READ		0x05
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
uint8_t recv_buffer[6]; //The biggest message received will have 6 bytes

/* TASK HANDLERS -------------------------------------------------------------*/
TaskHandle_t adc_task_handle;
TaskHandle_t LED_task_handle;
TaskHandle_t serial_comm_handle;

/* Task prototype functions --------------------------------------------------*/
void adc_task(void *param);
void serial_comm_task(void *param);
void led_task(void *param);

/* Queue Handlers ------------------------------------------------------------*/
QueueHandle_t adc_post_q; //Queue to post adc value to serial Task
QueueHandle_t LED_post_q; //Queue to post ack response to serial task
QueueHandle_t serial_post_adc_q; //Queue to post serial command to adc task
QueueHandle_t serial_post_led_q; //Queue to post led command to led task
QueueHandle_t serial_isr_q; //Queue to send flag from isr callback to serial task
							//This is need since i don't know in which core the callback function will
							//run when its called (probably the same core that register the isr)

//Since the task's dont share the same resource, there's no need for semaphore

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* Creating Queues ------------------------------------------------------------*/
  adc_post_q = xQueueCreate(5, sizeof(uint16_t));
  LED_post_q = xQueueCreate(5, sizeof(uint8_t));
  serial_post_adc_q = xQueueCreate(5, sizeof(uint8_t));
  serial_post_led_q = xQueueCreate(5, sizeof(uint8_t));
  serial_isr_q = xQueueCreate(5,sizeof(uint8_t));

  /* Creating Tasks ------------------------------------------------------------*/
  xTaskCreate(adc_task, "adc_task", 128, NULL, 2, &adc_task_handle); //Medium priority task
  xTaskCreate(led_task, "led_task", 128, NULL, 1, &LED_task_handle); //Lowest priority task
  xTaskCreate(serial_comm_task, "serial_task", 128, NULL, 3, &serial_comm_handle); //High priority task

  //Starting FreeRTOS scheduler
  vTaskStartScheduler();
  /* Infinite loop */
  while (1)
  {
	  //Just feeding task wtd
	  vTaskDelay(pdMS_TO_TICKS(100));
  }
}
/* Simple function to calculate checksum of data
 */
uint8_t checksum_calc(uint8_t *data, int length){
	int sum = 0, i = 0;
	while(i < length){
		sum += data[i];
		i ++;
	}
	return (uint8_t)sum;
}
// Every time data is received from ISR, this function is called.
// It's used to set a flag to serial comm task
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t flag_to_send = 1;
	// Just put a flag in queue to notify serial task that data has been received
	xQueueSend(serial_isr_q, &flag_to_send, portMAX_DELAY);
}

/* Serial communication task */
void serial_comm_task(void *param){
	uint8_t recv_data_flag, led_ack, command;
	uint8_t ack = 0x02; //ACk response to led command (wasn't specified in the
	uint8_t recv_size = 4;
	uint8_t send_buffer[6] = {0x01, 0x05, 0x02, 0x00, 0x00, 0x00};
	uint16_t adc_data;
	//REceive data through UART 2 until 4 bytes are received
	/* Format message:
	* 1º byte - 0x01
	* 2º byte - Command: 0x01 -> Turn On LED
	*				   0x02 -> Turn Off LED
	*				   0x03 -> LED Toggle
	*				   0x04 -> Message loop-back
	*				   0x05 -> read ADC
	* 3º byte - Quantity of parameters of next byte
	* 4º byte - Parameters (when needed)
	* last byte - checksum
	* When no parameters is present, the message will have 4 byte.
	* The max size of message is 6 (when reading the 12 bits ADC)
	*/
	HAL_UART_Receive_IT(&huart2, recv_buffer, recv_size);

	while(1){
		//received data flag set
		if(xQueueReceive(serial_isr_q, &recv_data_flag, pdMS_TO_TICKS(10)) == pdTRUE){
			//CHeck if the first byte of message is 0x01 and if the checksum of received data is valid
			if(recv_buffer[0] == MESSAGE_HEADER && recv_buffer[3]==checksum_calc(recv_buffer, 3)){
				// Check command byte
				command = recv_buffer[1];
				switch(command){
					case LED_ON:
					case LED_OFF:
					case LED_TOGGLE:
						//Send command to led task
						xQueueSend(serial_post_led_q, &command, portMAX_DELAY);
						break;
					case LOOP_BACK:
						//Send received command back through uart
						HAL_UART_Transmit_IT(&huart2, recv_buffer, recv_size);
						break;
					case ADC_READ:
						//Send received command to adc buffer
						xQueueSend(serial_post_adc_q, &command, portMAX_DELAY);
						break;
					default:
						break;
				}
			}
			else{
				//Since no treatment for error or corrupted message was asked, i will just leave a NOP here;
				__NOP();
			}
		}
		//received flag from led task, meaning command was acknowledge
		if(xQueueReceive(LED_post_q,&led_ack,pdMS_TO_TICKS(10)) == pdTRUE){
			if(led_ack == 1){
				HAL_UART_Transmit_IT(&huart2, &ack, 1);
			}
		}
		if(xQueueReceive(adc_post_q, &adc_data, pdMS_TO_TICKS(10))){
			send_buffer[3] = (uint8_t) (adc_data>>8); //MSB
			send_buffer[4] = (uint8_t) adc_data; 	  //LSB
			send_buffer[5] = checksum_calc(send_buffer, 5); //Calculate checksum
			HAL_UART_Transmit_IT(&huart2, send_buffer, 6);
		}
		//Just feeding task wtd
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
void adc_task(void *param){
	uint8_t adc_flag;
	uint16_t adc_value;
	while(1){
		if(xQueueReceive(serial_post_adc_q, &adc_flag, pdMS_TO_TICKS(10))==pdTRUE){
			// Starts AD conversion
			HAL_ADC_Start(&hadc1);
			// Block process until AD conversion is done
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			// Get raw value
			adc_value = (uint16_t) HAL_ADC_GetValue(&hadc1);
			// post value in the queue
			xQueueSend(adc_post_q, &adc_value, portMAX_DELAY);
		}
		//Just feeding task wtd
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
void led_task(void *param){
	uint8_t led_cmd;
	uint8_t ack_flag = 1;
	while(1){
		// led command was received
		if(xQueueReceive(serial_post_led_q, &led_cmd, pdMS_TO_TICKS(10))==pdTRUE){
			switch(led_cmd){
				//LED (LD2) GPIO is GPIOA
				//LED pin is GPIO_PIN_5
				case LED_ON:
					// Turn on LED
					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
					// Post flag to signal acknowledge
					xQueueSend(LED_post_q, &ack_flag, portMAX_DELAY);
					break;
				case LED_OFF:
					// Turn off LED
					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
					xQueueSend(LED_post_q, &ack_flag, portMAX_DELAY);
					break;
				case LED_TOGGLE:
					//Toggle LED
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					xQueueSend(LED_post_q, &ack_flag, portMAX_DELAY);
					break;
				default:
					break;
			}
		}
		//Just feeding task wtd
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
