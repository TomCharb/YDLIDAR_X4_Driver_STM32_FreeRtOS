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
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "drv_LIDAR.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ROWS_M1 2
#define COLS_M1 10
#define COLS_M2 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_LIDAR_t h_LIDAR; //Handel de notre LIDAR

//Mettre ca dans la structure
int frame_start = 0;
int frame_end = 0;

SemaphoreHandle_t SemHalfCallBack;
SemaphoreHandle_t SemClpCallBack;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	return chr;
}
int uart_transmit(uint8_t *p_data, uint16_t size){
	HAL_UART_Transmit(&huart1, p_data, size, HAL_MAX_DELAY);
	return 0;
}
int uart_it_transmit(uint8_t *p_data, uint16_t size){
	HAL_UART_Transmit_IT(&huart1, p_data, size);
	return 0;
}
int uart_dma_transmit(uint8_t *p_data, uint16_t size){
	HAL_UART_Transmit_DMA(&huart1, p_data, size);
	return 0;
}
int uart_poll_receive(uint8_t *p_data, uint16_t size){
	HAL_UART_Receive(&huart1, p_data, size, HAL_MAX_DELAY);
	return 0;
}
int uart_it_receive(uint8_t *p_data, uint16_t size){
	HAL_UART_Receive_IT(&huart1, p_data, size);
	return 0;
}
int uart_dma_receive(uint8_t *p_data, uint16_t size){
	HAL_UART_Receive_DMA(&huart1, p_data, size);
	return 0;
}

void TaskLIDAR (void * pvParameters){
	for(;;){
		xSemaphoreTake(SemHalfCallBack, portMAX_DELAY);
		for(int i=0; i<DATA_BUFF_SIZE; i++){
			if(i==DATA_BUFF_SIZE/2){
				xSemaphoreTake(SemClpCallBack, portMAX_DELAY);
			}
			//Réponce à la commande Scan
			if(i==0){
				if((h_LIDAR.data_buff[i]==0xA5) && (h_LIDAR.data_buff[i+1]==0x5A) && (h_LIDAR.data_buff[i+2]==0x05) && (h_LIDAR.data_buff[i+3]==0x00) && (h_LIDAR.data_buff[i+4]==0x00) && (h_LIDAR.data_buff[i+5]==0x40) && (h_LIDAR.data_buff[i+6]==0x81)){
					printf("Scan Command Reply\r\n");
					h_LIDAR.processing.idx=0;
					i=6;
					frame_start=7;
					frame_end=frame_start+4;
				}
			}

			if(i==frame_start){
				h_LIDAR.processing.PH=h_LIDAR.data_buff[i];
			}
			else if(i==frame_start+1){
				h_LIDAR.processing.PH=h_LIDAR.processing.PH|(h_LIDAR.data_buff[i]<<8);
			}
			else if(i==frame_start+2){
				h_LIDAR.processing.CT=h_LIDAR.data_buff[i];
			}
			else if(i==frame_start+3){
				frame_end=frame_start+9+2*h_LIDAR.data_buff[i];
				h_LIDAR.processing.LSN=h_LIDAR.data_buff[i];
			}
			else if(i==frame_start+4){
				h_LIDAR.processing.FSA=h_LIDAR.data_buff[i];
			}
			else if(i==frame_start+5){
				h_LIDAR.processing.FSA=h_LIDAR.processing.FSA|(h_LIDAR.data_buff[i]<<8);
			}
			else if(i==frame_start+6){
				h_LIDAR.processing.LSA=h_LIDAR.data_buff[i];
			}
			else if(i==frame_start+7){
				h_LIDAR.processing.LSA=h_LIDAR.processing.LSA|(h_LIDAR.data_buff[i]<<8);
			}
			else if(i==frame_start+8){
				h_LIDAR.processing.CS=h_LIDAR.data_buff[i];
			}
			else if(i==frame_start+9){
				h_LIDAR.processing.CS=h_LIDAR.processing.CS|(h_LIDAR.data_buff[i]<<8);
			}
			else if(i==frame_end){
				h_LIDAR.processing.frame_buff[h_LIDAR.processing.idx++]=h_LIDAR.data_buff[i];
				if(frame_end-frame_start>11){
					LIDAR_process_frame(&h_LIDAR); //On récupère les différents points
					medianFilter(&h_LIDAR); //On filtre ces points
					find_clusters(&h_LIDAR); //On regroupe ces points
				}
				h_LIDAR.processing.idx=0;
				frame_start=frame_end+1;
				frame_end=frame_start+5;
			}
			else{
				h_LIDAR.processing.frame_buff[h_LIDAR.processing.idx++]=h_LIDAR.data_buff[i];
			}
		}
		frame_start=frame_start-DATA_BUFF_SIZE;
		frame_end=frame_end-DATA_BUFF_SIZE;


//		printf("#\r\n");
//		for(int i=0 ; i<360/2 ; i++){
//			//printf("%d\r\n",h_LIDAR.processing.point_buff[2*i]); //On envoi les valeurs
//			//printf("%d\r\n",h_LIDAR.processing.filtred_buff[2*i]); //On envoi les valeurs filtrées
//		}
		printf("#\r\n"); //Envoi des clusters
		for(int i=0 ; i<h_LIDAR.processing.cluster_cnt ; i++){
			printf("%d,%d\r\n",h_LIDAR.processing.clusters[i].angle_moyen,h_LIDAR.processing.clusters[i].distance_moyenne); //On envoi les valeurs
		}
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
	TaskHandle_t xHandleLIDAR = NULL;
	BaseType_t ret;

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	h_LIDAR.serial_drv.transmit=uart_transmit;
	h_LIDAR.serial_drv.it_transmit=uart_it_transmit;
	h_LIDAR.serial_drv.dma_transmit=uart_dma_transmit;
	h_LIDAR.serial_drv.poll_receive=uart_poll_receive;
	h_LIDAR.serial_drv.it_receive=uart_it_receive;
	h_LIDAR.serial_drv.dma_receive=uart_dma_receive;

	printf("\r\n===== Driver_LIDAR =====\r\n");

	LIDAR_stop(&h_LIDAR);
	HAL_Delay(1000);
	LIDAR_get_info(&h_LIDAR);
	LIDAR_get_health_stat(&h_LIDAR);

	ret = xTaskCreate(TaskLIDAR,"TaskLIDAR",STACK_SIZE,(void *) NULL,1,&xHandleLIDAR);
	if (ret != pdPASS)
	{
		printf("Error creating TaskLIDAR\r\n");
		Error_Handler();
	}
	printf("Task LIDAR created\r\n");

	SemHalfCallBack = xSemaphoreCreateBinary();
	SemClpCallBack = xSemaphoreCreateBinary();

	LIDAR_start(&h_LIDAR);
	vTaskStartScheduler();


	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(SemHalfCallBack,&higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(SemClpCallBack,&higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
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
