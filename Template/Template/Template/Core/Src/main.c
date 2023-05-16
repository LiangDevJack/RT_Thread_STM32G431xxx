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
#include "rtthread.h"
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	key_chack,
	key_prerss,
	key_release,
	key_over
}key_start;
key_start keystart = key_chack;
uint8_t key_value;
uint32_t key_press_time = 20;

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* 定义全局变量 */
uint16_t LED_ALL = 0XFFFF;
uint8_t show;
extern uint16_t R37_Vol,R38_Vol;

/* 定义线程控制块 */
static rt_thread_t key_thread = RT_NULL;          //按键扫描
static void key_thread_entry(void *parameter);

static rt_thread_t KeyScan_thread = RT_NULL;      //按键获取
static void KeyScan_thread_entry(void *parameter);

static rt_thread_t send_thread = RT_NULL;         //消息发送
static void send_thread_entry(void* parameter);

static rt_thread_t LED_thread = RT_NULL;          //LED闪烁
static void LED_thread_entry(void* parameter);

/* 定义软件定时器控制块 */
static rt_timer_t swtmr_adc = RT_NULL;            //软件定时器
static void swtmr_adc_callback(void *parameter);

/* 定义消息队列控制块 */
static rt_mq_t adc_mq = RT_NULL;

/* 定义互斥量控制块 */
static rt_mutex_t test_mutex = RT_NULL;

/* 定义事件控制块 */
static rt_event_t key_event = RT_NULL;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
	
	LED_Close();
  /* USER CODE BEGIN 2 */

  adc_mq = rt_mq_create("adc_mq",40,20,RT_IPC_FLAG_FIFO);       //消息队列
	
	key_event = rt_event_create("key_event", RT_IPC_FLAG_FIFO);   //事件
	rt_event_send(key_event, key2_event);
	
	test_mutex = rt_mutex_create("test_mutex",RT_IPC_FLAG_FIFO);  //互斥量
	
	KeyScan_thread =  rt_thread_create( "KeyScan_thread",KeyScan_thread_entry,RT_NULL,256,3,20);
	rt_thread_startup(KeyScan_thread);
	
	key_thread =  rt_thread_create( "key_thread",key_thread_entry,RT_NULL,256,3,20);  
	rt_thread_startup(key_thread);	
	
  send_thread = rt_thread_create( "send",send_thread_entry,RT_NULL,256,3,20); 
	rt_thread_startup(send_thread);
	
	swtmr_adc = rt_timer_create("swtmr_adc_callback",swtmr_adc_callback,0,1000,RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(swtmr_adc);
	
	LED_thread = rt_thread_create( "KeyScan_thread",LED_thread_entry,RT_NULL,256,3,20);
	rt_thread_startup(LED_thread);			
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

static void KeyScan_thread_entry(void *parameter)
{
	while(1)
	{
		switch(keystart)
		{
			case key_chack :
			{
				if(B1 == 0 | B2 == 0 | B3 == 0 |B4 == 0)
				{
					keystart = key_prerss ;
				}
			}break;
			case key_prerss :
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
					keystart = key_chack;
				}
			}break;
			default : break;
		}
		rt_thread_delay(10);
	}
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

		}
		else if(recved == key2_event)
		{
      rt_mutex_take(test_mutex,10);
			if(test_mutex->hold < 255)
			{
				if(test_mutex->hold % 2 == 0)
				{
					rt_kprintf("界面1：电压\n");
          show = 0;
				}
				else
				{
					rt_kprintf("界面2：阈值\n");
					show = 1;
				}
			}
		}		
		else if(recved == key3_event)
		{
      BUZZ_ON;
		}	
		else if(recved == key4_event)
		{
      BUZZ_OFF;
		}			
		rt_thread_delay(10);
	}
}

static void swtmr_adc_callback(void* parameter)
{
	uint16_t r37_queue;
	uint16_t r38_queue;
	
	rt_mq_recv(adc_mq, &r37_queue, sizeof(r37_queue), 0);
	rt_mq_recv(adc_mq, &r38_queue, sizeof(r38_queue), 0);
	
  if(show == 0)
	{
		rt_kprintf("r37: %d\n", r37_queue);
		if (r37_queue < 1500)
		{
			rt_kprintf("level 1\n");
			rt_event_send(key_event, key5_event);
		}
		else if(r37_queue > 1500 && r37_queue < 2500)
		{
			rt_kprintf("level 2\n");
			rt_event_send(key_event, key6_event);
		}
		else 
		{
			rt_kprintf("level 3\n");
			rt_event_send(key_event, key7_event);
		}
	}
	else if(show == 1)
	{
		rt_kprintf("level 1: %d\n",1500);
		rt_kprintf("level 3: %d\n",2500);
	}
}

static void send_thread_entry(void *parameter)
{
	uint16_t R37_data;
	uint16_t R38_data;
	while(1)
	{
		ADC_Get();
		R37_data = R37_Vol;
		R38_data = R38_Vol;

		rt_mq_send(adc_mq, &R37_data, sizeof(R37_data));		
		rt_mq_send(adc_mq, &R38_data, sizeof(R38_data));	
		
		rt_thread_delay(1000);
	}
}

static void LED_thread_entry(void* parameter)
{
	rt_uint32_t LED_Run;
	while(1)
	{
		rt_event_recv(key_event, key5_event|key6_event|key7_event, 
						RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &LED_Run);
		
		if(LED_Run == key5_event)
		{
      LED_ON(2);
			rt_thread_delay(100);
			LED_OFF(2);
			rt_thread_delay(100);
		}
		else if(LED_Run == key6_event)
		{
      LED_Close();
		}		
		else if(LED_Run == key7_event)
		{
      LED_ON(3);
			rt_thread_delay(100);
			LED_OFF(3);
			rt_thread_delay(100);
		}	
    rt_thread_delay(10);		
	}
}



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
