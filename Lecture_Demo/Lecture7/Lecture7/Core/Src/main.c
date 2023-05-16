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
static rt_thread_t receive_thread = RT_NULL;
static rt_thread_t send_thread = RT_NULL;
//定义消息队列控制块
static rt_mq_t test_mq = RT_NULL;

static void receive_thread_entry(void* parameter);
static void send_thread_entry(void* parameter);

//定义信号量控制块
static rt_sem_t test_sem = RT_NULL;
//全局变量
uint8_t ucValue[2]={0x00, 0x00};

//定义按键标志信号量控制块

static rt_sem_t KeyFlag_sem = RT_NULL;



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
	
//	 rt_kprintf("这是一个RTT 二值信号量同步实验！\n");
//  rt_kprintf("同步成功输出Successful，否则输出Fail\n");
	
//	 rt_kprintf("这是一个RTT 计数信号量实验！\n");
//	rt_kprintf("车位默认5个，按下K1申请车位，按下K2释放车位！\n");
	
	//实验目的显示
	rt_kprintf("这是一个RTT 二值信号量同步实验！\n");
	rt_kprintf("通过二值信号量对按键状态进行标记！\n");
	
	 	 KeyFlag_sem = rt_sem_create("KeyFlag_sem",	/*信号量的名字*/
						1,	/*信号量初始值，默认只有一个信号量*/
						RT_IPC_FLAG_FIFO 	/* 队列模式FIFO(0x00)*/
						);
   if(KeyFlag_sem != RT_NULL)
		rt_kprintf("按键标志信号量创建成功！\n\n");
   
    	 test_mq = rt_mq_create("test_mq",	/*消息队列的名字*/
						40,	/*消息的最大长度*/
						20,	/*消息队列的最大容量*/
						RT_IPC_FLAG_FIFO 	/* 队列模式FIFO(0x00)*/
						);
   if(test_mq != RT_NULL)
		rt_kprintf("消息队列创建成功！\n\n");
	


//	 test_sem = rt_sem_create("test_sem",	/*信号量的名字*/
//						5,	/*信号量初始值，默认只有一个信号量*/
//						RT_IPC_FLAG_FIFO 	/* 队列模式FIFO(0x00)*/
//						);
//   if(test_sem != RT_NULL)
//		rt_kprintf("信号量创建成功！\n\n");

   
   	   receive_thread =                          /* 线程控制块指针 */
			rt_thread_create( "receive",              /* 线程名字 */
							  receive_thread_entry,   /* 线程入口函数 */
							  RT_NULL,             /* 线程入口函数参数 */
							  512,                 /* 线程栈大小 */
							  3,                   /* 线程的优先级 */
							  20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
	if (receive_thread != RT_NULL)
		rt_thread_startup(receive_thread);
	else
		return -1;

	
		   send_thread =                          /* 线程控制块指针 */
		rt_thread_create( "send",              /* 线程名字 */
						  send_thread_entry,   /* 线程入口函数 */
						  RT_NULL,             /* 线程入口函数参数 */
						  512,                 /* 线程栈大小 */
						  2,                   /* 线程的优先级 */
						  20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
	if (send_thread != RT_NULL)
		rt_thread_startup(send_thread);
	else
		return -1;

	
}

//int message_queue(void)
//{
//	rt_kprintf("这是一个RTT 消息队列实验！\n");
//  rt_kprintf("按下K1 或者K2 发送队列消息\n");
//  rt_kprintf("receive 线程接收到消息在串口回显\n");

//	
//	 test_mq = rt_mq_create("test_mq",	/*消息队列的名字*/
//						40,	/*消息的最大长度*/
//						20,	/*消息队列的最大容量*/
//						RT_IPC_FLAG_FIFO 	/* 队列模式FIFO(0x00)*/
//						);
//   if(test_mq != RT_NULL)
//		rt_kprintf("消息队列创建成功！\n\n");

//   
//   	   receive_thread =                          /* 线程控制块指针 */
//			rt_thread_create( "receive",              /* 线程名字 */
//							  receive_thread_entry,   /* 线程入口函数 */
//							  RT_NULL,             /* 线程入口函数参数 */
//							  512,                 /* 线程栈大小 */
//							  3,                   /* 线程的优先级 */
//							  20);                 /* 线程时间片 */
//                   
//    /* 启动线程，开启调度 */
//	if (receive_thread != RT_NULL)
//		rt_thread_startup(receive_thread);
//	else
//		return -1;

//	
//		   send_thread =                          /* 线程控制块指针 */
//		rt_thread_create( "send",              /* 线程名字 */
//						  send_thread_entry,   /* 线程入口函数 */
//						  RT_NULL,             /* 线程入口函数参数 */
//						  512,                 /* 线程栈大小 */
//						  2,                   /* 线程的优先级 */
//						  20);                 /* 线程时间片 */
//                   
//    /* 启动线程，开启调度 */
//	if (send_thread != RT_NULL)
//		rt_thread_startup(send_thread);
//	else
//		return -1;
//	
//	return 0;
//}
//MSH_CMD_EXPORT(message_queue, message queue sample);

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

static void send_thread_entry(void* parameter)
{	
	rt_err_t uwRet = RT_EOK;
	
	while (1)
	{	
		
		uint32_t send_data1=  getADC_R37();
		uint32_t send_data2 = getADC_R38();
		static rt_err_t result;
		
		result = rt_sem_take(KeyFlag_sem, 0);
		
		if (result==RT_EOK)
		{					
			
			if (KEY_Value==1) 
				{					
					uwRet = rt_mq_send(test_mq,
										&send_data1,
										sizeof(send_data1));
					if(RT_EOK != uwRet)
					{
						rt_kprintf("数据不能发送到消息队列！错误代码： 0x%1x\n", uwRet);
					}
				
				}
			else if(KEY_Value==2)
			{
					uwRet = rt_mq_send(test_mq,
										&send_data2,
										sizeof(send_data2));
					if(RT_EOK != uwRet)
					{
						rt_kprintf("数据不能发送到消息队列！错误代码： 0x%1x\n", uwRet);
					}
			}			
		}
	rt_thread_mdelay(20);
	}
}

//// 计数型信号量
//static void receive_thread_entry(void* parameter)
//{	
//	rt_err_t uwRet = RT_EOK;
//	
//	while (1)
//	{
//		if ((KeyFlag==1)&&(KEY_Value==1))
//		{
//			KeyFlag=0;			
//						
//			uwRet = rt_sem_take(test_sem, 0);
//			if(RT_EOK == uwRet)
//			{
//				rt_kprintf("KEY1被按下：成功申请到车位。\r\n");
//			}
//			else
//			{
//				rt_kprintf("KEY1被按下：不好意思，现在停车场已满！\r\n");
//			}				
//						
//		}
//		rt_thread_delay(20);
//	}
//}

//static void send_thread_entry(void* parameter)
//{	
//	rt_err_t uwRet = RT_EOK;
//	while (1)
//	{
//		if ((KeyFlag==1)&&(KEY_Value==2))
//		{
//			KeyFlag=0;	
//			
//			uwRet = rt_sem_release(test_sem);
//			if(RT_EOK == uwRet)
//			{
//				rt_kprintf("KEY2被按下：释放一个停车位。\r\n");
//			}
//			else
//			{
//				rt_kprintf("KEY2被按下：没有车位可以释放！\r\n");
//			}
//						
//		}
//	rt_thread_mdelay(20);
//	}
//}


//static void receive_thread_entry(void* parameter)
//{	
//	while (1)
//	{
//		rt_sem_take(test_sem,
//					RT_WAITING_FOREVER);
//		if(ucValue[0]==ucValue[1])
//		{
//			rt_kprintf("Successful\n");
//		}
//		else
//		{
//			rt_kprintf("Fail\n");
//		}
//		rt_sem_release(test_sem);
//		rt_thread_delay(1000);
//	}
//}

//static void send_thread_entry(void* parameter)
//{	
//	while (1)
//	{
//		rt_sem_take(test_sem, RT_WAITING_FOREVER);
//		ucValue[0]++;
//		rt_thread_delay(100);
//		ucValue[1]++;
//		rt_sem_release(test_sem);
//		rt_thread_yield();	
//	}
//}


//static void receive_thread_entry(void* parameter)
//{	
//	rt_err_t uwRet = RT_EOK;	

//	while (1)
//	{
//		rt_kprintf("接收线程尝试获取信号量！\n");
//		uwRet = rt_sem_take(test_sem,
//					RT_WAITING_FOREVER);
//		
//		if(RT_EOK == uwRet)
//		{
//			rt_kprintf("接收线程获取信号量成功！\n");
//		}
//		else
//		{
//			rt_kprintf("接收线程获取信号量失败！\n");
//		}
//		
//		
//		if(ucValue[0]==ucValue[1])
//		{
//			rt_kprintf("Successful\n");
//		}
//		else
//		{
//			rt_kprintf("Fail\n");
//		}
//		rt_sem_release(test_sem);
//		rt_thread_delay(100);
//	}
//}

//static void send_thread_entry(void* parameter)
//{	
//	rt_err_t uwRet = RT_EOK;		
//	while (1)
//	{		
//		rt_kprintf("发送线程尝试获取信号量！\n");
//		rt_sem_take(test_sem, RT_WAITING_FOREVER);
//		if(RT_EOK == uwRet)
//		{
//			rt_kprintf("发送线程获取信号量成功！\n");
//		}
//		else
//		{
//			rt_kprintf("发送线程获取信号量失败！\n");
//		}		
//		
//		ucValue[0]++;
//		rt_kprintf("ucValue[0]: %d \n", ucValue[0]);
//		rt_kprintf("ucValue[1]: %d \n", ucValue[1]);
//		rt_kprintf("发送线程延时3秒，模拟占用！\n");
//		rt_thread_delay(3000);
//		ucValue[1]++;
//		rt_kprintf("ucValue[0]: %d \n", ucValue[0]);
//		rt_kprintf("ucValue[1]: %d \n", ucValue[1]);
//		rt_sem_release(test_sem);
//		rt_thread_yield();	
//	}
//}


static void receive_thread_entry(void* parameter)
{	
	rt_err_t uwRet = RT_EOK;
	uint32_t r_queue;
	while (1)
	{
		uwRet = rt_mq_recv(test_mq,
							&r_queue,
							sizeof(r_queue),
							RT_WAITING_FOREVER);
		if (RT_EOK == uwRet)
		{
			rt_kprintf("本次接收到的数据是： %d\n", r_queue);
		}
		else
		{
			rt_kprintf("数据接收出错，错误代码：0x%1x\n", uwRet);
		}
		rt_thread_delay(200);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		KeyScan(); //调用按键扫描函数
	}
}

//static void send_thread_entry(void* parameter)
//{	
//	rt_err_t uwRet = RT_EOK;
//	
//	while (1)
//	{	
//		
//		uint32_t send_data1=  getADC_R37();
//		uint32_t send_data2 = getADC_R38();
//		
//		if (KeyFlag==1)
//		{
//			KeyFlag=0;			
//			
//			if (KEY_Value==1) 
//				{					
//					uwRet = rt_mq_send(test_mq,
//										&send_data1,
//										sizeof(send_data1));
//					if(RT_EOK != uwRet)
//					{
//						rt_kprintf("数据不能发送到消息队列！错误代码： 0x%1x\n", uwRet);
//					}
//				
//				}
//			else if(KEY_Value==2)
//			{
//					uwRet = rt_mq_send(test_mq,
//										&send_data2,
//										sizeof(send_data2));
//					if(RT_EOK != uwRet)
//					{
//						rt_kprintf("数据不能发送到消息队列！错误代码： 0x%1x\n", uwRet);
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
					rt_sem_release(KeyFlag_sem);
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
