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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <rtthread.h>
#include "stdio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	key_check,
	key_press,
	key_release,
	key_over
}key_start;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define B1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define B2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define B3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define B4 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

#define BUZZ_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define BUZZ_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define key1_event (0x01 << 1)
#define key2_event (0x01 << 2)
#define key3_event (0x01 << 3)
#define key4_event (0x01 << 4)
#define key5_event (0x01 << 5)
#define key6_event (0x01 << 6)
#define key7_event (0x01 << 7)
#define key8_event (0x01 << 8)
#define key9_event (0x01 << 9)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
key_start keystart = key_check;
uint8_t key_value;
uint32_t key_press_time = 20;
char str[20];
uint8_t interface = 1;
uint8_t gata = 1;
uint16_t LED_ALL = 0XFFFF;

/* 定义线程控制块 */
static rt_thread_t send_thread = RT_NULL;         //消息发送
static rt_thread_t LED_thread = RT_NULL;          //LED闪烁
static rt_thread_t key_thread = RT_NULL;          //按键扫描

static rt_thread_t KeyScan_thread = RT_NULL;      //按键获取
static void KeyScan_thread_entry(void *parameter);
/* 定义事件控制块 */
static rt_event_t key_event = RT_NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Close(void);
void LED_ON(uint8_t n);
void LED_OFF(uint8_t n);
static void key_thread_entry(void *parameter);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t getADC_R37(void)
{
	uint16_t adc = 0;
	
	HAL_ADC_Start(&hadc2); 
	HAL_ADC_PollForConversion(&hadc2,2);
	adc = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	
	return adc;
}

uint16_t getADC_R38(void)
{
	uint16_t adc = 0;
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,2);
	adc = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	
	return adc;
}


static void send_thread_entry(void *parameter)
{
	uint16_t R37_data;
	uint16_t R38_data;
	while(1)
	{
		if(interface)
		{
			R37_data = getADC_R37();
			R38_data = getADC_R38();
			rt_kprintf("R37: %d\n", R37_data);
			rt_kprintf("R38: %d\n", R38_data);
		}
		else rt_kprintf("阈值: %d\n", gata);
		sprintf(str,"  r37: %d  ",R37_data);
		LCD_DisplayStringLine(Line3,(unsigned char *)str);
		
		sprintf(str,"  r38: %d  ",R38_data);
		LCD_DisplayStringLine(Line5,(unsigned char *)str);

//		rt_mq_send(adc_mq, &R37_data, sizeof(R37_data));		
//		rt_mq_send(adc_mq, &R38_data, sizeof(R38_data));	
		
		rt_thread_delay(1000);
	}
}

static void LED_thread_entry(void* parameter)
{
	
	
	while(1)
	{
		LED_ON(1);
		BUZZ_OFF;
		rt_thread_mdelay(500);
		
//		BUZZ_OFF;
		rt_thread_mdelay(500);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	rt_hw_board_init();
	BUZZ_OFF;
	LED_OFF(1);
	LED_Close();
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetTextColor(Blue);
	LCD_SetBackColor(Black); 
	LCD_DisplayStringLine(Line0,(unsigned char *)"       TEST         ");
	LCD_DisplayStringLine(Line1,(unsigned char *)"     RT_Thread      ");
	LCD_DisplayStringLine(Line2,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line3,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line4,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line5,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line6,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line7,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line8,(unsigned char *)"                    ");
	LCD_DisplayStringLine(Line9,(unsigned char *)"                    ");

	rt_kprintf("RT_Thread综合测试仅限203内部交流\n");
	rt_kprintf("     切记：    禁止传阅！\n");
	rt_kprintf("RT_Thread综合测试仅限203内部交流\n");
	rt_kprintf("     切记：    禁止传阅！\n");
	rt_kprintf("RT_Thread综合测试仅限203内部交流\n");
	rt_kprintf("     切记：    禁止传阅！\n");
	rt_kprintf("     切记：    禁止传阅！\n");
	rt_kprintf("     切记：    禁止传阅！\n");
	rt_kprintf("     切记：    禁止传阅！\n");
	rt_kprintf("*********************************\n");
	rt_kprintf("*********************************\n");
	rt_kprintf("*********************************\n");
	rt_kprintf("     如果同意：\n");
	rt_kprintf("     请输入命令StartTest,开始测试！\n");
	
}

void StartTest(void)
{
	key_event = rt_event_create("key_event", RT_IPC_FLAG_FIFO);   //事件
	rt_event_send(key_event, key2_event);
	
	key_thread =  rt_thread_create( "key_thread",key_thread_entry,RT_NULL,256,3,20);  
	rt_thread_startup(key_thread);	
	
	KeyScan_thread =  rt_thread_create( "KeyScan_thread",KeyScan_thread_entry,RT_NULL,256,3,20);
	rt_thread_startup(KeyScan_thread);
	
	send_thread = rt_thread_create( "send",send_thread_entry,RT_NULL,256,3,20); 
	rt_thread_startup(send_thread);
	LED_thread  = rt_thread_create( "LED",LED_thread_entry,RT_NULL,256,3,20); 
	rt_thread_startup(LED_thread);
}
MSH_CMD_EXPORT(StartTest, Test Project);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void LED_Close(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
	                        |GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void LED_ON(uint8_t n)
{
	LED_ALL &= ((0XFEFF << (n - 1)) | (0XFEFF >> (16 - n + 1)));
	GPIOC->ODR = LED_ALL;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void LED_OFF(uint8_t n)
{
	LED_ALL |= ((0X0100 << (n - 1)) | (0X0100 >> (16 - n + 1)));
	GPIOC->ODR = LED_ALL;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

static void key_thread_entry(void *parameter)
{
	rt_uint32_t recved;
	
	while(1)
	{
		rt_event_recv(key_event, key1_event|key2_event|key3_event|key4_event, 
						RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		if(recved == key1_event)
		{
			interface = !interface;
			if(interface)
			rt_kprintf("界面1：电压\n");
			else rt_kprintf("界面2：阈值\n");
		}
		else if(recved == key2_event)
		{
      
			
		}		
		else if(recved == key3_event)
		{
      BUZZ_ON;
			rt_kprintf("蜂鸣器响\n");
		}	
		else if(recved == key4_event)
		{
			rt_kprintf("蜂鸣器灭\n");
      BUZZ_OFF;
		}			
		rt_thread_delay(10);
	}
}

static void KeyScan_thread_entry(void *parameter)
{
	while(1)
	{
		switch(keystart)
		{
			case key_check :
			{
				if(B1 == 0 | B2 == 0 | B3 == 0 |B4 == 0)
				{
					keystart = key_press ;
				}
			}break;
			case key_press :
			{
				if(B1 == 0)rt_event_send(key_event, key1_event);
				else if(B2 == 0)rt_event_send(key_event, key2_event);
				else if(B3 == 0)rt_event_send(key_event, key3_event);
				else if(B4 == 0)rt_event_send(key_event, key4_event);
				keystart = key_release ;
			}break;
			case key_release :
			{
				if(B1 == 0 | B2 == 0 | B3 == 0 |B4 == 0)
				{
					key_press_time = key_press_time + 10;
				}
				else 
				{
					keystart = key_check;
				}
			}break;
			default : break;
		}
		rt_thread_delay(10);
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
