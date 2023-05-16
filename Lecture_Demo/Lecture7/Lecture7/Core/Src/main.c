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

/* �����߳̿��ƿ� */
static rt_thread_t receive_thread = RT_NULL;
static rt_thread_t send_thread = RT_NULL;
//������Ϣ���п��ƿ�
static rt_mq_t test_mq = RT_NULL;

static void receive_thread_entry(void* parameter);
static void send_thread_entry(void* parameter);

//�����ź������ƿ�
static rt_sem_t test_sem = RT_NULL;
//ȫ�ֱ���
uint8_t ucValue[2]={0x00, 0x00};

//���尴����־�ź������ƿ�

static rt_sem_t KeyFlag_sem = RT_NULL;



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
	
//	 rt_kprintf("����һ��RTT ��ֵ�ź���ͬ��ʵ�飡\n");
//  rt_kprintf("ͬ���ɹ����Successful���������Fail\n");
	
//	 rt_kprintf("����һ��RTT �����ź���ʵ�飡\n");
//	rt_kprintf("��λĬ��5��������K1���복λ������K2�ͷų�λ��\n");
	
	//ʵ��Ŀ����ʾ
	rt_kprintf("����һ��RTT ��ֵ�ź���ͬ��ʵ�飡\n");
	rt_kprintf("ͨ����ֵ�ź����԰���״̬���б�ǣ�\n");
	
	 	 KeyFlag_sem = rt_sem_create("KeyFlag_sem",	/*�ź���������*/
						1,	/*�ź�����ʼֵ��Ĭ��ֻ��һ���ź���*/
						RT_IPC_FLAG_FIFO 	/* ����ģʽFIFO(0x00)*/
						);
   if(KeyFlag_sem != RT_NULL)
		rt_kprintf("������־�ź��������ɹ���\n\n");
   
    	 test_mq = rt_mq_create("test_mq",	/*��Ϣ���е�����*/
						40,	/*��Ϣ����󳤶�*/
						20,	/*��Ϣ���е��������*/
						RT_IPC_FLAG_FIFO 	/* ����ģʽFIFO(0x00)*/
						);
   if(test_mq != RT_NULL)
		rt_kprintf("��Ϣ���д����ɹ���\n\n");
	


//	 test_sem = rt_sem_create("test_sem",	/*�ź���������*/
//						5,	/*�ź�����ʼֵ��Ĭ��ֻ��һ���ź���*/
//						RT_IPC_FLAG_FIFO 	/* ����ģʽFIFO(0x00)*/
//						);
//   if(test_sem != RT_NULL)
//		rt_kprintf("�ź��������ɹ���\n\n");

   
   	   receive_thread =                          /* �߳̿��ƿ�ָ�� */
			rt_thread_create( "receive",              /* �߳����� */
							  receive_thread_entry,   /* �߳���ں��� */
							  RT_NULL,             /* �߳���ں������� */
							  512,                 /* �߳�ջ��С */
							  3,                   /* �̵߳����ȼ� */
							  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (receive_thread != RT_NULL)
		rt_thread_startup(receive_thread);
	else
		return -1;

	
		   send_thread =                          /* �߳̿��ƿ�ָ�� */
		rt_thread_create( "send",              /* �߳����� */
						  send_thread_entry,   /* �߳���ں��� */
						  RT_NULL,             /* �߳���ں������� */
						  512,                 /* �߳�ջ��С */
						  2,                   /* �̵߳����ȼ� */
						  20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
	if (send_thread != RT_NULL)
		rt_thread_startup(send_thread);
	else
		return -1;

	
}

//int message_queue(void)
//{
//	rt_kprintf("����һ��RTT ��Ϣ����ʵ�飡\n");
//  rt_kprintf("����K1 ����K2 ���Ͷ�����Ϣ\n");
//  rt_kprintf("receive �߳̽��յ���Ϣ�ڴ��ڻ���\n");

//	
//	 test_mq = rt_mq_create("test_mq",	/*��Ϣ���е�����*/
//						40,	/*��Ϣ����󳤶�*/
//						20,	/*��Ϣ���е��������*/
//						RT_IPC_FLAG_FIFO 	/* ����ģʽFIFO(0x00)*/
//						);
//   if(test_mq != RT_NULL)
//		rt_kprintf("��Ϣ���д����ɹ���\n\n");

//   
//   	   receive_thread =                          /* �߳̿��ƿ�ָ�� */
//			rt_thread_create( "receive",              /* �߳����� */
//							  receive_thread_entry,   /* �߳���ں��� */
//							  RT_NULL,             /* �߳���ں������� */
//							  512,                 /* �߳�ջ��С */
//							  3,                   /* �̵߳����ȼ� */
//							  20);                 /* �߳�ʱ��Ƭ */
//                   
//    /* �����̣߳��������� */
//	if (receive_thread != RT_NULL)
//		rt_thread_startup(receive_thread);
//	else
//		return -1;

//	
//		   send_thread =                          /* �߳̿��ƿ�ָ�� */
//		rt_thread_create( "send",              /* �߳����� */
//						  send_thread_entry,   /* �߳���ں��� */
//						  RT_NULL,             /* �߳���ں������� */
//						  512,                 /* �߳�ջ��С */
//						  2,                   /* �̵߳����ȼ� */
//						  20);                 /* �߳�ʱ��Ƭ */
//                   
//    /* �����̣߳��������� */
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
						rt_kprintf("���ݲ��ܷ��͵���Ϣ���У�������룺 0x%1x\n", uwRet);
					}
				
				}
			else if(KEY_Value==2)
			{
					uwRet = rt_mq_send(test_mq,
										&send_data2,
										sizeof(send_data2));
					if(RT_EOK != uwRet)
					{
						rt_kprintf("���ݲ��ܷ��͵���Ϣ���У�������룺 0x%1x\n", uwRet);
					}
			}			
		}
	rt_thread_mdelay(20);
	}
}

//// �������ź���
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
//				rt_kprintf("KEY1�����£��ɹ����뵽��λ��\r\n");
//			}
//			else
//			{
//				rt_kprintf("KEY1�����£�������˼������ͣ����������\r\n");
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
//				rt_kprintf("KEY2�����£��ͷ�һ��ͣ��λ��\r\n");
//			}
//			else
//			{
//				rt_kprintf("KEY2�����£�û�г�λ�����ͷţ�\r\n");
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
//		rt_kprintf("�����̳߳��Ի�ȡ�ź�����\n");
//		uwRet = rt_sem_take(test_sem,
//					RT_WAITING_FOREVER);
//		
//		if(RT_EOK == uwRet)
//		{
//			rt_kprintf("�����̻߳�ȡ�ź����ɹ���\n");
//		}
//		else
//		{
//			rt_kprintf("�����̻߳�ȡ�ź���ʧ�ܣ�\n");
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
//		rt_kprintf("�����̳߳��Ի�ȡ�ź�����\n");
//		rt_sem_take(test_sem, RT_WAITING_FOREVER);
//		if(RT_EOK == uwRet)
//		{
//			rt_kprintf("�����̻߳�ȡ�ź����ɹ���\n");
//		}
//		else
//		{
//			rt_kprintf("�����̻߳�ȡ�ź���ʧ�ܣ�\n");
//		}		
//		
//		ucValue[0]++;
//		rt_kprintf("ucValue[0]: %d \n", ucValue[0]);
//		rt_kprintf("ucValue[1]: %d \n", ucValue[1]);
//		rt_kprintf("�����߳���ʱ3�룬ģ��ռ�ã�\n");
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
			rt_kprintf("���ν��յ��������ǣ� %d\n", r_queue);
		}
		else
		{
			rt_kprintf("���ݽ��ճ���������룺0x%1x\n", uwRet);
		}
		rt_thread_delay(200);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		KeyScan(); //���ð���ɨ�躯��
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
//						rt_kprintf("���ݲ��ܷ��͵���Ϣ���У�������룺 0x%1x\n", uwRet);
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
//						rt_kprintf("���ݲ��ܷ��͵���Ϣ���У�������룺 0x%1x\n", uwRet);
//					}
//			}			
//		}
//	rt_thread_mdelay(20);
//	}
//}


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
