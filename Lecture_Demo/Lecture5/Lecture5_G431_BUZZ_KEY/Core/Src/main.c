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

// //线程1--动态线程控制LED
///* 定义线程控制块 */
//static rt_thread_t led1_thread = RT_NULL;

// //线程2--动态线程检测按键
///* 定义线程控制块 */
//static rt_thread_t key_thread = RT_NULL;


//static void led1_thread_entry(void* parameter);
//static void key_thread_entry(void* parameter);


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
static void led3_control (void* parameter)
{	
	rt_uint8_t count;
	for(count = 0 ; count < 10 ;count++)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		rt_kprintf("comtrol led3, led3 on\r\n");
		rt_thread_delay(300);   /* 延时500个tick */
	
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		rt_kprintf(" comtrol led3, led3 off\r\n");
		rt_thread_delay(300);   /* 延时500个tick */	 		
	}
}
MSH_CMD_EXPORT(led3_control, led3 on and off 10 times);

void Open_LED1(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	rt_kprintf("LED1 open!\n");
}
MSH_CMD_EXPORT(Open_LED1, Open first LED);
void Close_LED1(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	rt_kprintf("LED1 close!\n");
}
MSH_CMD_EXPORT(Close_LED1, Close first LED);



static void Operate_LED(int argc, char **argv)
{
	if(argc<2)
	{
		rt_kprintf("Please input 'Light_LED<1|2|3|4|5|6|7|8>'\n");
		return;
	}
	
	if(!rt_strcmp(argv[1], "1"))
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		rt_kprintf("操作LED1!\n");
	}
	else if(!rt_strcmp(argv[1], "2"))
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		rt_kprintf("操作LED2!\n");
	}
	else if(!rt_strcmp(argv[1], "3"))
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		rt_kprintf("操作LED3!\n");
	}
	else if(!rt_strcmp(argv[1], "4"))
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
		rt_kprintf("操作LED4!\n");
	}
	else if(!rt_strcmp(argv[1], "5"))
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
		rt_kprintf("操作LED5!\n");
	}
	else
	{
		rt_kprintf("Please input 'Light_LED<1|2|3|4|5|6|7|8>'\n");
		
	}
		
	
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(Operate_LED, Operate LED as you want!);


#define THREAD_STACK_SIZE   1024
#define THREAD_PRIORITY     20
#define THREAD_TIMESLICE    10
/* 针对每个线程的计数器 */
volatile rt_uint32_t count[2];

/* 线程 1、2 共用一个入口，但入口参数不同 */
static void thread_entry(void* parameter)
{
    rt_uint32_t value;

    value = (rt_uint32_t)parameter;
    while (1)
    {
        rt_kprintf("thread %d is running\n", value);
        rt_thread_mdelay(1000); // 延时一段时间
    }
}

static rt_thread_t tid1 = RT_NULL;
static rt_thread_t tid2 = RT_NULL;

static void hook_of_scheduler(struct rt_thread* from, struct rt_thread* to)
{
    rt_kprintf("from: %s -->  to: %s \n", from->name , to->name);
}

int scheduler_hook(void)
{
    /* 设置调度器钩子 */
    rt_scheduler_sethook(hook_of_scheduler);

    /* 创建线程 1 */
    tid1 = rt_thread_create("thread1",
                            thread_entry, (void*)1,
                            THREAD_STACK_SIZE,
                            THREAD_PRIORITY, THREAD_TIMESLICE);
    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);

    /* 创建线程 2 */
    tid2 = rt_thread_create("thread2",
                            thread_entry, (void*)2,
                            THREAD_STACK_SIZE,
                            THREAD_PRIORITY,THREAD_TIMESLICE - 5);
    if (tid2 != RT_NULL)
        rt_thread_startup(tid2);
    return 0;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(scheduler_hook, scheduler_hook sample);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//static void led1_thread_entry(void* parameter)
//{	
//	while (1)
//	{
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//		rt_kprintf("led_thread running, led1 on\r\n");
//		rt_thread_delay(300);   /* 延时500个tick */	
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//		rt_kprintf("led_thread running, led1 off\r\n");
//		rt_thread_delay(300);   /* 延时500个tick */	 		
//	}
//}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == htim6.Instance)
//	{
//		KeyScan(); //调用按键扫描函数
//	}
//}

//static void key_thread_entry(void* parameter)
//{	
//	rt_err_t uwRet = RT_EOK;
//	while (1)
//	{
//		if (KeyFlag==1)
//		{
//			KeyFlag=0;			
//			
//			if (KEY_Value==1) 
//				{
//					rt_kprintf("挂起LED1线程！\n");
//					uwRet = rt_thread_suspend(led1_thread);
//					if(RT_EOK==uwRet)
//					{
//						rt_kprintf("挂起LED1线程成功！\n");
//					}
//					else
//					{
//						rt_thread_delay(3000);
//						while(1)
//						{
//							
//							rt_kprintf("挂起LED1线程失败！失败代码： 0x%1x\n", uwRet);
//							uwRet = rt_thread_suspend(led1_thread);
//							if(RT_EOK==uwRet)
//							{
//								rt_kprintf("挂起LED1线程成功！\n");
//								break;
//							}
//						}
//					}
//				}
//			else if(KEY_Value==2)
//			{
//					rt_kprintf("恢复LED1线程！\n");
//					uwRet = rt_thread_resume(led1_thread);
//					if(RT_EOK==uwRet)
//					{
//						rt_kprintf("恢复LED1线程成功！\n");
//					}
//					else
//					{
//						rt_kprintf("恢复LED1线程失败！失败代码： 0x%1x\n", uwRet);
//					}
//			}			
//		}

//	rt_thread_mdelay(20);
//	}
//}

////===========================四个按键，按键1可以根据按键时间对应2个不同的功能=================================
void KeyScan(void)
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
//					KeyFlag = 1; //设置按键有效标志
					
					if (B1==GPIO_PIN_RESET) KEY_Value=1;
					else if (B2==GPIO_PIN_RESET) KEY_Value=2;
					else if (B3==GPIO_PIN_RESET) KEY_Value=3;
					else if (B4==GPIO_PIN_RESET) KEY_Value=4;
					
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
					KeyFlag = 1; //设置按键有效标志
				}
				break;
				
			}
			default: break;
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
