/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtthread.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	KEY_UP = 0,//按键抬起
	KEY_DEBOUNCE,//按键消抖
	KEY_WAIT_RELEASE//按键等待释放
}KEY_STATE;


/* 定义线程控制块 */
static rt_thread_t key_thread = RT_NULL;
static rt_thread_t KeyScan_thread = RT_NULL;
//定义事件控制块
static rt_event_t key_event = RT_NULL;
#define key1_event (0x01 << 1)
#define key2_event (0x01 << 2)
#define key3_event (0x01 << 3)

static void key_thread_entry(void *parameter);
static void KeyScan_thread_entry(void *parameter);



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// GPIO输入输出宏定义
#define B1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define B2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)
#define B3 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define B4 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

#define BUZZ_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define BUZZ_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define LED1_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
#define LED1_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t KEY_Value=0; //按键值
KEY_STATE KeyState=KEY_UP;  //按键状态指示变量，初值为按键抬起状态
volatile uint8_t KeyFlag=0;  //按键有效标志，0：无效；1：有效
uint32_t KEY_PressTime=20;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KeyScan(void);


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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

	HAL_TIM_Base_Start_IT(&htim6);

}

int event_sample(void)
{
	rt_kprintf("这是一个RTT-按键事件标志组同步实验！\n");
	rt_kprintf("利用事件标记按键事件的发生！\n");
	
	rt_kprintf("准备创建按键事件！\n");
	key_event = rt_event_create("key_event", RT_IPC_FLAG_FIFO);
	
	if (key_event != RT_NULL)
	{
		rt_kprintf("按键事件创建成功！\n");
	}

		   key_thread =                          /* 线程控制块指针 */
			rt_thread_create( "key_thread",              /* 线程名字 */
							  key_thread_entry,   /* 线程入口函数 */
							  RT_NULL,             /* 线程入口函数参数 */
							  256,                 /* 线程栈大小 */
							  3,                   /* 线程的优先级 */
							  20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
	if (key_thread != RT_NULL)
		rt_thread_startup(key_thread);
	else
		return -1;

		   KeyScan_thread =                          /* 线程控制块指针 */
		rt_thread_create( "KeyScan_thread",              /* 线程名字 */
						  KeyScan_thread_entry,   /* 线程入口函数 */
						  RT_NULL,             /* 线程入口函数参数 */
						  256,                 /* 线程栈大小 */
						  2,                   /* 线程的优先级 */
						  20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
	if (KeyScan_thread != RT_NULL)
		rt_thread_startup(KeyScan_thread);
	else
		return -1;


}
MSH_CMD_EXPORT(event_sample, key event sample);

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void key_thread_entry(void *parameter)
{
	rt_uint32_t recved;

	while(1)
	{
		rt_event_recv(key_event, key1_event|key2_event, 
						RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		rt_kprintf("获取事件\n");
		if(recved == key1_event)
		{
			rt_kprintf("按键1按下，获取按键1事件成功！\n 翻转LED1的电平\n");
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		}
		else if(recved == key2_event)
		{
			rt_kprintf("按键2按下，获取按键2事件成功！\n 翻转LED3的电平\n");
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		}
		else 
		{
			rt_kprintf("事件错误！\n");
		}
		rt_thread_delay(20);
	}
}

static void KeyScan_thread_entry(void *parameter)
{
	while(1)
	{
		switch(KeyState)
		{
			case KEY_UP: //按键抬起状态
			{
				//读到低电平，转换到按键消抖状态
				if((B1==GPIO_PIN_RESET)||(B2==GPIO_PIN_RESET)||(B3==GPIO_PIN_RESET)||(B4==GPIO_PIN_RESET))
				{
					KeyState= KEY_DEBOUNCE;
				}
				break;
			}
			
			case KEY_DEBOUNCE: //按键消抖状态
			{
				// 读到低电平，转换到按键等待等待释放状态，并设置按键有效标志
				if ((B1==GPIO_PIN_RESET)||(B2==GPIO_PIN_RESET)||(B3==GPIO_PIN_RESET)||(B4==GPIO_PIN_RESET))
				{
					KeyState = KEY_WAIT_RELEASE;
			
					if (B1==GPIO_PIN_RESET) rt_event_send(key_event, key1_event);
					else if (B2==GPIO_PIN_RESET) rt_event_send(key_event, key2_event);
//					else if (B3==GPIO_PIN_RESET) rt_event_send(key_event, key3_event);
//					else if (B4==GPIO_PIN_RESET) KEY_Value=4;
					
				}
				//读到高电平，表明是干扰信号，转换到按键抬起状态
				else
				{
					KeyState = KEY_UP; 
				}		
							
				break;
			}
			case KEY_WAIT_RELEASE: //按键等待释放状态
			{
				
				//读到低电平，说明按键仍然处于按下的状态，按键时间增加10ms
				if((B1==GPIO_PIN_RESET)||(B2==GPIO_PIN_RESET)||(B3==GPIO_PIN_RESET)||(B4==GPIO_PIN_RESET))
				{
					KEY_PressTime = KEY_PressTime +10;
				}
				//读到高电平，说明按键释放，转换到按键抬起状态
				else 
				{
					KeyState = KEY_UP;
//					rt_sem_release(KeyFlag_sem);					
				}
				break;
				
			}
			default: break;
		}
		rt_thread_delay(20);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
