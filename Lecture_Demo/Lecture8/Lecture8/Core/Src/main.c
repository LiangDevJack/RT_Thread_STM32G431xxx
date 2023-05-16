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
	KEY_UP = 0,//����̧��
	KEY_DEBOUNCE,//��������
	KEY_WAIT_RELEASE//�����ȴ��ͷ�
}KEY_STATE;


//���尴����־�ź������ƿ�

static rt_sem_t KeyFlag_sem = RT_NULL;


//�μ�ʵ��2
/* �����߳̿��ƿ� */
static rt_thread_t plus1_thread = RT_NULL;
static rt_thread_t plus2_thread = RT_NULL;
//���廥�������ƿ�
static rt_mutex_t dynamic_mutex = RT_NULL;
//ȫ�ֱ���
static rt_uint8_t number1,number2 = 0;

static void plus1_thread_entry(void* parameter);
static void plus2_thread_entry(void* parameter);


//�μ�ʵ��3
/* �����߳̿��ƿ� */
static rt_thread_t tid1 = RT_NULL;
static rt_thread_t tid2 = RT_NULL;
static rt_thread_t tid3 = RT_NULL;
//���廥�������ƿ�
static rt_mutex_t mutex = RT_NULL;

static void thread1_entry(void *parameter);
static void thread2_entry(void *parameter);
static void thread3_entry(void *parameter);



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// GPIO��������궨��
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

uint8_t KEY_Value=0; //����ֵ
KEY_STATE KeyState=KEY_UP;  //����״ָ̬ʾ��������ֵΪ����̧��״̬
volatile uint8_t KeyFlag=0;  //������Ч��־��0����Ч��1����Ч
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



	
	 	 KeyFlag_sem = rt_sem_create("KeyFlag_sem",	/*�ź���������*/
						1,	/*�ź�����ʼֵ��Ĭ��ֻ��һ���ź���*/
						RT_IPC_FLAG_FIFO 	/* ����ģʽFIFO(0x00)*/
						);
   if(KeyFlag_sem != RT_NULL)
		rt_kprintf("������־�ź��������ɹ���\n\n");
   

	
}

int mutex_sample2(void)
{
	   
 dynamic_mutex = rt_mutex_create("dynamic_mutex", RT_IPC_FLAG_FIFO );
   if(dynamic_mutex != RT_NULL)
		rt_kprintf("�����������ɹ���\n\n");
   
   
	 	   plus1_thread =                          /* �߳̿��ƿ�ָ�� */
			rt_thread_create( "plus1_thread",              /* �߳����� */
							  plus1_thread_entry,   /* �߳���ں��� */
							  RT_NULL,             /* �߳���ں������� */
							  512,                 /* �߳�ջ��С */
							  3,                   /* �̵߳����ȼ� */
							  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (plus1_thread != RT_NULL)
		rt_thread_startup(plus1_thread);
	else
		return -1;

	
			   plus2_thread =                          /* �߳̿��ƿ�ָ�� */
		rt_thread_create( "plus2_thread",              /* �߳����� */
						  plus2_thread_entry,   /* �߳���ں��� */
						  RT_NULL,             /* �߳���ں������� */
						  512,                 /* �߳�ջ��С */
						  3,                   /* �̵߳����ȼ� */
						  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (plus2_thread != RT_NULL)
		rt_thread_startup(plus2_thread);
	else
		return -1;	
	
	return 0;
}
MSH_CMD_EXPORT(mutex_sample2, mutex sample2);

int pri_inversion(void)
{
		rt_kprintf("����һ��RTT ������ͬ��ʵ�飡\n");
    rt_kprintf("����3����ͬ���ȼ����̣߳������л��������߳��Ƿ񱻵������ȴ��߳����ȼ��е�������ȼ���\n");


	
mutex = rt_mutex_create("mutex", RT_IPC_FLAG_FIFO);
   
   if (mutex == RT_NULL)
   {
		rt_kprintf("create dynamic mutex failded.\n");
	   return -1;
   }
   
	 tid1 =                          /* �߳̿��ƿ�ָ�� */
			rt_thread_create( "thread1",              /* �߳����� */
							  thread1_entry,   /* �߳���ں��� */
							  RT_NULL,             /* �߳���ں������� */
							  512,                 /* �߳�ջ��С */
							  5,                   /* �̵߳����ȼ� */
							  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (tid1 != RT_NULL)
		rt_thread_startup(tid1);
	else
		return -1;

	 tid2 =                          /* �߳̿��ƿ�ָ�� */
			rt_thread_create( "thread2",              /* �߳����� */
							  thread2_entry,   /* �߳���ں��� */
							  RT_NULL,             /* �߳���ں������� */
							  512,                 /* �߳�ջ��С */
							  6,                   /* �̵߳����ȼ� */
							  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (tid2 != RT_NULL)
		rt_thread_startup(tid2);
	else
		return -1;

	
	 tid3 =                          /* �߳̿��ƿ�ָ�� */
			rt_thread_create( "thread3",              /* �߳����� */
							  thread3_entry,   /* �߳���ں��� */
							  RT_NULL,             /* �߳���ں������� */
							  512,                 /* �߳�ջ��С */
							  7,                   /* �̵߳����ȼ� */
							  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (tid3 != RT_NULL)
		rt_thread_startup(tid3);
	else
		return -1;	
	return 0;

}
MSH_CMD_EXPORT(pri_inversion, priority inversion sample);

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

static void thread1_entry(void *parameter)
{
	rt_thread_delay(100);
	
	if(tid2->current_priority != tid3->current_priority)
	{
		rt_kprintf("the priority of thread2 is : %d\n", tid2->current_priority);
		rt_kprintf("the priority of thread3 is : %d\n", tid3->current_priority);
		rt_kprintf("test failed.\n");
		return;
	}
	else
	{
		rt_kprintf("the priority of thread2 is : %d\n", tid2->current_priority);
		rt_kprintf("the priority of thread3 is : %d\n", tid3->current_priority);
		rt_kprintf("test ok.\n");
		return;
	}
}

static void thread2_entry(void *parameter)
{
	rt_err_t result;
	rt_kprintf("the priority of thread2 is : %d\n", tid2->current_priority);
	
	rt_thread_delay(50);
	
	result = rt_mutex_take(mutex, RT_WAITING_FOREVER);
	
	if(result == RT_EOK)
	{
		rt_mutex_release(mutex);
	}
}


static void thread3_entry(void *parameter)
{
	rt_tick_t tick;
	rt_err_t result;
	
	rt_kprintf("the priority of thread3 is : %d\n", tid3->current_priority);
			
	result = rt_mutex_take(mutex, RT_WAITING_FOREVER);
	
	if(result != RT_EOK)
	{
		rt_kprintf("thread3 take mutex failed.\n");
	}
	tick = rt_tick_get();
	while(rt_tick_get() - tick < (RT_TICK_PER_SECOND / 2));
	
	rt_mutex_release(mutex);
}

static void plus1_thread_entry(void* parameter)
{
	while(1)
	{
		rt_mutex_take(dynamic_mutex, RT_WAITING_FOREVER);
		number1++;
		rt_thread_delay(10);
		number2++;
		rt_mutex_release(dynamic_mutex);
	}
}

static void plus2_thread_entry(void* parameter)
{
	while(1)
	{
		rt_mutex_take(dynamic_mutex, RT_WAITING_FOREVER);
		if(number1!=number2)
		{
			rt_kprintf("not protect. numner1 = %d, number2 = %d \n", number1, number2);
		}
		else
		{
			rt_kprintf("mutex protect, number1 = number2 is %d\n", number1);
		}
		
		number1=number1+2;		
		number2=number2+2;
		rt_mutex_release(dynamic_mutex);
		
		if(number1>=50)
			return;
	}
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		KeyScan(); //���ð���ɨ�躯��
	}
}




////===========================�ĸ�����������1���Ը��ݰ���ʱ���Ӧ2����ͬ�Ĺ���=================================
void KeyScan(void)
{
		switch(KeyState)
		{
			case KEY_UP: //����̧��״̬
			{
				//�����͵�ƽ��ת������������״̬
				if((B1==GPIO_PIN_RESET)||(B2==GPIO_PIN_RESET)||(B3==GPIO_PIN_RESET)||(B4==GPIO_PIN_RESET))
				{
					KeyState= KEY_DEBOUNCE;
				}
				break;
			}
			
			case KEY_DEBOUNCE: //��������״̬
			{
				// �����͵�ƽ��ת���������ȴ��ȴ��ͷ�״̬�������ð�����Ч��־
				if ((B1==GPIO_PIN_RESET)||(B2==GPIO_PIN_RESET)||(B3==GPIO_PIN_RESET)||(B4==GPIO_PIN_RESET))
				{
					KeyState = KEY_WAIT_RELEASE;
//					KeyFlag = 1; //���ð�����Ч��־
					
					if (B1==GPIO_PIN_RESET) KEY_Value=1;
					else if (B2==GPIO_PIN_RESET) KEY_Value=2;
					else if (B3==GPIO_PIN_RESET) KEY_Value=3;
					else if (B4==GPIO_PIN_RESET) KEY_Value=4;
					
				}
				//�����ߵ�ƽ�������Ǹ����źţ�ת��������̧��״̬
				else
				{
					KeyState = KEY_UP; 
				}		
							
				break;
			}
			case KEY_WAIT_RELEASE: //�����ȴ��ͷ�״̬
			{
				
				//�����͵�ƽ��˵��������Ȼ���ڰ��µ�״̬������ʱ������10ms
				if((B1==GPIO_PIN_RESET)||(B2==GPIO_PIN_RESET)||(B3==GPIO_PIN_RESET)||(B4==GPIO_PIN_RESET))
				{
					KEY_PressTime = KEY_PressTime +10;
				}
				//�����ߵ�ƽ��˵�������ͷţ�ת��������̧��״̬
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
