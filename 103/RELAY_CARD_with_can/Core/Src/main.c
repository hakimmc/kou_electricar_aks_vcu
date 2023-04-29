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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

CAN_HandleTypeDef hcan;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can_task */
osThreadId_t can_taskHandle;
const osThreadAttr_t can_task_attributes = {
  .name = "can_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for triggered_relay */
osThreadId_t triggered_relayHandle;
const osThreadAttr_t triggered_relay_attributes = {
  .name = "triggered_relay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t pTxMailbox;
uint8_t RxData;
uint8_t Txdata[0];
uint8_t CanReceive=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartCan(void *argument);
void Startrelay(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data[8];
uint8_t compressed[2]={0,0};

int binarray_to_decimal(uint8_t array[8]){
	int cnt=0;
	for(int i=0;i<8;i++){
		cnt+= (array[i]*(1<<i));
	}
	return cnt;
}
int WaitForMission(uint8_t dest_id,uint8_t id,int timeout){
	int t_out=0;
	int r_val;
	//int durum=0;
	while(1){
		HAL_CAN_AddTxMessage(&hcan, &pTxHeader, compressed, &pTxMailbox);
		uint8_t rxData[1];
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRxHeader, rxData);
		if(rxData[0]==id && (dest_id==pRxHeader.StdId || !dest_id)){
			r_val = 1;
			break;
		}
		t_out++;
		HAL_Delay(1);
		if(t_out==timeout){
			r_val = 0;
			break;
		}
	}
	return r_val;
}
uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
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
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);   //start can communication

  //HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // enable interrupt
  pTxHeader.DLC = 1;
  pTxHeader.IDE = CAN_ID_STD;
  pTxHeader.RTR = CAN_RTR_DATA;
  pTxHeader.StdId = 0x12;

  //set filter parameters
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x000<< 5;
  sFilterConfig.FilterIdLow = 0x000 << 5;
  sFilterConfig.FilterMaskIdHigh = 0x000 << 5;
  sFilterConfig.FilterMaskIdLow = 0x000 << 5;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;


  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of can_task */
  can_taskHandle = osThreadNew(StartCan, NULL, &can_task_attributes);

  /* creation of triggered_relay */
  triggered_relayHandle = osThreadNew(Startrelay, NULL, &triggered_relay_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT7_Pin|OUT6_Pin|OUT5_Pin|OUT4_Pin
                          |OUT3_Pin|OUT2_Pin|OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN3_Pin IN4_Pin IN5_Pin
                           IN6_Pin IN7_Pin IN8_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN3_Pin|IN4_Pin|IN5_Pin
                          |IN6_Pin|IN7_Pin|IN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT8_Pin */
  GPIO_InitStruct.Pin = OUT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT7_Pin OUT6_Pin OUT5_Pin OUT4_Pin
                           OUT3_Pin OUT2_Pin OUT1_Pin */
  GPIO_InitStruct.Pin = OUT7_Pin|OUT6_Pin|OUT5_Pin|OUT4_Pin
                          |OUT3_Pin|OUT2_Pin|OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  uint16_t adc_val = HAL_ADC_GetValue(&hadc1); // gas pedal adc variable to motor drivers with can
	  HAL_ADC_Stop(&hadc1);

	  if(!HAL_GPIO_ReadPin (IN1_GPIO_Port, IN1_Pin))
	  {
		  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 0);
		  data[0] = 0;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 1);
		  data[0] = 1;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN2_GPIO_Port, IN2_Pin))
	  {
		 data[1] = 1;
	  }
	  else
	  {
		 data[1] = 0;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN3_GPIO_Port, IN3_Pin))
	  {
		data[2] = 1;
	  }
	  else
	  {
		data[2] = 0;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN4_GPIO_Port, IN4_Pin))
	  {
		data[3] = 1;
	  }
	  else
	  {
		data[3] = 0;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN5_GPIO_Port, IN5_Pin))
	  {
		data[4] = 1;
	  }
	  else
	  {
		data[4] = 0;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN6_GPIO_Port, IN6_Pin))
	  {
		 data[5] = 1;
	  }
	  else
	  {
		 data[5] = 0;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN7_GPIO_Port, IN7_Pin))
	  {
		  data[6] = 1;
	  }
	  else
	  {
		  data[6] = 0;
	  }
	  //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	  if(!HAL_GPIO_ReadPin (IN8_GPIO_Port, IN8_Pin))
	  {
		 data[7] = 1;
	  }
	  else
	  {
		 data[7] = 0;
	  }
	  compressed[0] = binarray_to_decimal(data);
	  compressed[1] = MAP(adc_val, 0, 4095, 0, 255);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCan */
/**
* @brief Function implementing the can_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCan */
void StartCan(void *argument)
{
  /* USER CODE BEGIN StartCan */
	pTxHeader.StdId = 0x12;
	uint8_t id_ = pTxHeader.StdId;
	pTxHeader.DLC = 2;
  /* Infinite loop */
  for(;;)
  {
	  	  //feedback func for can
		WaitForMission(0,id_,100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		//i put some delay because of if can bus transmit go fast, feedback timeout will be so short;
		HAL_Delay(50);
  }
  /* USER CODE END StartCan */
}

/* USER CODE BEGIN Header_Startrelay */
/**
* @brief Function implementing the triggered_relay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startrelay */
void Startrelay(void *argument)
{
  /* USER CODE BEGIN Startrelay */

	// tip : our task is just to trigger 2 relays in the same sequence, anothers working like normal

	uint16_t clock_=0; // clock her 500 degerinde 0a cekip sistemi sinyaller icin *delaysiz kontrol eder
	uint8_t left_signal_flag=0;
	uint8_t right_signal_flag=0;
	uint8_t signal_flag=0;

  /* Infinite loop */
	//INPUTS: 1-stop; 2-dörtlü; 3-sol; 4-sag; 5-far; 6-silecek; 7-korna; 8-switch;
	//OUTPUTS : 1-?; 2-sol; 3-?; 4-far; 5-silecek; 6-korna; 7-? 8-sag;
  for(;;)
  {

	  //OTHERS SO SIMPLE THEY DONT CONTAIN TOGGLE FOR RELAY
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

	  //INPUT 1 //STOP
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[0]==1)
	  {
		 HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 1);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 1

	  //INPUT 2 //DORTLU
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[1]==1)
	  {
		  signal_flag=1;
	  }
	  else
	  {
		  signal_flag=0;
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 2

	  /*                    !? TOGGLE ?!				  */
	  //INPUT 3 //SOL
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[2]==1 || signal_flag)
	  {
		  left_signal_flag=1;
		  if(clock_<101){
			 HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, 1);
		  }
		  else{
			 HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, 0);
		  }
	  }
	  else
	  {
		  left_signal_flag=0;
		  HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT1_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 3

	  /*                    !? TOGGLE ?!				  */
	  //INPUT 4 //SAG
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[3]==1 || signal_flag)
	  {
		  right_signal_flag=1;
		  if(clock_<101){
			 HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, 1);
		  }
		  else{
			 HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, 0);
		  }
	  }
	  else
	  {
		  right_signal_flag=0;
		  HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 4

	  //INPUT 5 //FAR
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[4]==1)
	  {
		  HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, 1);
		  //HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, 1);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, 0);
		  //HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 5

	  //INPUT 6 //SILECEK
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[5]==1)
	  {
		 HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, 1);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 6

	  //INPUT 7 //KORNA
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[6]==1)
	  {
		 HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, 1);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 7

	  /*INPUT 8 //SWITCH
	  //MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	  if(data[7]==1)
	  {
		 HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, 1);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, 0);
	  }
	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
	  //INPUT 8*/

	  //WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW


	  //CLOCK TIME!!!!!!!
	  if(right_signal_flag || left_signal_flag){
		  clock_++;

		  if(clock_>200){//if >500 reset clock time
			  clock_=0;
		  }
	  }
	  else{
		  clock_=0;
	  }
	  HAL_Delay(1);
  }
  /* USER CODE END Startrelay */
}

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
