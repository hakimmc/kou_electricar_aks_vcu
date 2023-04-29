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

/*________________________________External Libs_______________________________*/

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
/*----------------------------------------------------------------------------*/
#include "electricar.h"
#include "i2c-lcd.h"
/*____________________________________________________________________________*/

/*________________________________CAN Includes________________________________*/

CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t pTxMailbox;
uint8_t RxData[8];
/*____________________________________________________________________________*/

/*___________________RF VARIABLES, COMPRESSED TEMPS & CONSTS__________________*/

char arr[300];
char tempp[25][4];
const float max_battery=100.8;
char myMessagex[50];
/*____________________________________________________________________________*/

/*______________________________AKS MAIN STRUCTER_____________________________*/

struct struct1{
	struct struct2{
		float battery_voltage[24];
		//float battery_temparature;
		int battery_temparature;
		uint8_t bms_error_code;
		float max_battery;
		float min_battery;
		int sum_battery;
	}BMS;
	struct struct3{
		int charge_watt;
		int time;
		uint8_t voltage_error;
		uint8_t current_error;
		uint8_t temperature_error;
		uint8_t ReadyToFlow;
		uint8_t start_charge_state;
	}CHARGER;
	struct struct4{
		int vehicle_speed: 1;
	}DRIVER;
	struct struct5{
		uint8_t stop_;
		uint8_t hazard_;
		uint8_t left_;
		uint8_t right_;
		uint8_t light_;
		uint8_t wiper_;
		uint8_t horn_;
		char direction_;
	}RELAY;

}ecar;

/*____________________________________________________________________________*/

/*___________________________________NEXTION__________________________________*/

/*PAGE1*/
#define power "t9" //%
#define speed "t10"
#define temp "t11" //°43
#define watt "t0"
#define charge_percentage "t12"
#define charge_time "t5" //sn
#define state_of_charge "p6"
#define left_arrow "p9"
#define right_arrow "p17"
#define horn "p11"
#define lights "p13"
#define wiper "p15"

/*PAGE2*/
#define cell1 "x0"
#define cell2 "x1"
#define cell3 "x2"
#define cell4 "x3"
#define cell5 "x4"
#define cell6 "x5"
#define cell7 "x6"
#define cell8 "x7"
#define cell9 "x8"
#define cell10 "x9"
#define cell11 "x10"
#define cell12 "x11"
#define cell13 "x12"
#define cell14 "x13"
#define cell15 "x14"
#define cell16 "x15"
#define cell17 "x16"
#define cell18 "x17"
#define cell19 "x18"
#define cell20 "x19"
#define cell21 "x20"
#define cell22 "x21"
#define cell23 "x22"
#define cell24 "x23"
#define min_cell "x24"
#define max_cell "x25"

/*PAGE3*/
#define charge_code "t71"
#define voltage_err_code "p81"
#define current_err_code "p83"
#define temperature_err_code "p85"
/*____________________________________________________________________________*/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Vrefin_cal ((uint16_t *)((uint32_t)0x1FFF7A2A))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for nextion */
osThreadId_t nextionHandle;
const osThreadAttr_t nextion_attributes = {
  .name = "nextion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rf */
osThreadId_t rfHandle;
const osThreadAttr_t rf_attributes = {
  .name = "rf",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can */
osThreadId_t canHandle;
const osThreadAttr_t can_attributes = {
  .name = "can",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dac */
osThreadId_t dacHandle;
const osThreadAttr_t dac_attributes = {
  .name = "dac",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void start_nextion(void *argument);
void start_rf(void *argument);
void start_can(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */
/*______________________________External Functions____________________________*/
int prepare_data(){
	sprintf(arr,"%c%c",ecar.RELAY.direction_,ci2a(ecar.DRIVER.vehicle_speed));
	for(int j=0;j<24;j++){
		strcat(arr,cf2a(ecar.BMS.battery_voltage[j],tempp[j]));
		if(j==23){
			strcat(arr,cf2a((float)ecar.BMS.battery_temparature,tempp[j+1]));
		}
	}
	strcat(arr,"1\0");
	return 1;
}

int dest[8];
void decimal_to_bin(uint8_t number){ // EX for 255 = [1][1][1][1][1][1][1][1] binary code
    for(int i=0;i<8;i++){
        dest[i] = number%2;
        number/=2;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char M[20];
int fan_level=0;
int temp_of_lcd;

void GET_ADC_VALUE()
{
	if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
	{
		uint16_t adc_vall = HAL_ADC_GetValue(&hadc1);
		temp_of_lcd = (adc_vall*330) / 4095;
	}
}

void SEND_DATA_TO_LCD(){

    lcd_init();
    lcd_send_cmd(0x80);
    sprintf(&M[0],"TEMP %cC   : %d%cC", (char)223,ecar.BMS.battery_temparature,(char)223);
    lcd_send_string(&M[0]);
    HAL_Delay(100);

	sprintf(&M[0],"FAN LEVEL : %s", fan_level==0?"----":(fan_level==25?"#---":(fan_level==50?"##--":(fan_level==75?"###-":"####"))));
	lcd_send_cmd(0xC0);
	lcd_send_string(&M[0]);
	HAL_Delay(100);
}

int WaitForMission(uint8_t dest_id,uint8_t id,int timeout,uint8_t message){ // can send with feedback func
	int t_out=0;
	int r_val;
	//int durum=0;
	while(1){
		HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, &message, &pTxMailbox);
		uint8_t rxData[1];
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, rxData);
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
void CalculateMaxBattery(){
	int max=ecar.BMS.battery_voltage[0];
	for(int n=1;n<24;n++){
		if(max<ecar.BMS.battery_voltage[n]){max=ecar.BMS.battery_voltage[n];}
	}
	ecar.BMS.max_battery=max;
}
void CalculateMinBattery(){
	int min=ecar.BMS.battery_voltage[0];
	for(int n=1;n<24;n++){
		if(min>ecar.BMS.battery_voltage[n]){min=ecar.BMS.battery_voltage[n];}
	}
	ecar.BMS.min_battery=min;
}
/*____________________________________________________________________________*/

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_ADC_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  TIM2->CCR3 = 25; // ex pwm signal for fan level


  ecar.CHARGER.start_charge_state = 'I';
  //HAL_UART_Receive_DMA (&huart6, ecar.CHARGER.start_charge_state, 1);

  pTxHeader.DLC = 1;
  pTxHeader.IDE = CAN_ID_STD;
  pTxHeader.RTR = CAN_RTR_DATA;
  pTxHeader.StdId = 0x10;

   //set filter parameters
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.FilterBank = 0;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterIdHigh = 0x0000 ;
   sFilterConfig.FilterIdLow = 0x0000 ;
   sFilterConfig.FilterMaskIdHigh = 0x0000 ;
   sFilterConfig.FilterMaskIdLow = 0x0000 ;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

   HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

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
  /* creation of nextion */
  nextionHandle = osThreadNew(start_nextion, NULL, &nextion_attributes);

  /* creation of rf */
  rfHandle = osThreadNew(start_rf, NULL, &rf_attributes);

  /* creation of can */
  canHandle = osThreadNew(start_can, NULL, &can_attributes);

  /* creation of dac */
  dacHandle = osThreadNew(StartTask04, NULL, &dac_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_nextion */
/**
  * @brief  Function implementing the nextion thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_nextion */
void start_nextion(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(ecar.CHARGER.start_charge_state=='S'){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  }

	  //page1
	  //NEXTION_SEND(huart6, 0, charge_code, ecar.BMS.bms_error_code, myMessagex);

	  ecar.BMS.sum_battery=0;
	  for(int k=0;k<24;k++){
		 ecar.BMS.sum_battery+=ecar.BMS.battery_voltage[k]*1000;
	  }
	  NEXTION_SEND(huart6, 2, power,ecar.BMS.sum_battery/((int)(max_battery*1000)), myMessagex);
	  NEXTION_SEND(huart6, 2, speed, ecar.DRIVER.vehicle_speed, myMessagex);
	  NEXTION_SEND(huart6, 2, temp, ecar.BMS.battery_temparature, myMessagex);
	  NEXTION_SEND(huart6, 2, watt, ecar.CHARGER.charge_watt, myMessagex);
	  //NEXTION_SEND(huart6, 2, charge_percentage, number, myMessagex);
	  NEXTION_SEND(huart6, 0, state_of_charge, ecar.CHARGER.ReadyToFlow, myMessagex);
	  NEXTION_SEND(huart6, 2, charge_time, ecar.CHARGER.time, myMessagex);
	  /*NEXTION_	SEND(huart6, 0, left_arrow, status, myMessagex);
	  NEXTION_SEND(huart6, 0, right_arrow, status, myMessagex);
	  NEXTION_SEND(huart6, 0, horn, status, myMessagex);
	  NEXTION_SEND(huart6, 0, lights, status, myMessagex);
	  NEXTION_SEND(huart6, 0, wiper, status, myMessagex);*/

	  //page2
	  NEXTION_SEND(huart6, 1, cell1, (int)(ecar.BMS.battery_voltage[0]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell2, (int)(ecar.BMS.battery_voltage[1]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell3, (int)(ecar.BMS.battery_voltage[2]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell4, (int)(ecar.BMS.battery_voltage[3]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell5, (int)(ecar.BMS.battery_voltage[4]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell6, (int)(ecar.BMS.battery_voltage[5]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell7, (int)(ecar.BMS.battery_voltage[6]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell8, (int)(ecar.BMS.battery_voltage[7]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell9, (int)(ecar.BMS.battery_voltage[8]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell10, (int)(ecar.BMS.battery_voltage[9]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell11, (int)(ecar.BMS.battery_voltage[10]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell12, (int)(ecar.BMS.battery_voltage[11]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell13, (int)(ecar.BMS.battery_voltage[12]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell14, (int)(ecar.BMS.battery_voltage[13]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell15, (int)(ecar.BMS.battery_voltage[14]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell16, (int)(ecar.BMS.battery_voltage[15]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell17, (int)(ecar.BMS.battery_voltage[16]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell18, (int)(ecar.BMS.battery_voltage[17]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell19, (int)(ecar.BMS.battery_voltage[18]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell20, (int)(ecar.BMS.battery_voltage[19]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell21, (int)(ecar.BMS.battery_voltage[20]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell22, (int)(ecar.BMS.battery_voltage[21]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell23, (int)(ecar.BMS.battery_voltage[22]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, cell24, (int)(ecar.BMS.battery_voltage[23]*1000), myMessagex);
	  NEXTION_SEND(huart6, 1, max_cell, ecar.BMS.max_battery, myMessagex);
	  NEXTION_SEND(huart6, 1, min_cell, ecar.BMS.min_battery, myMessagex);

	  //page3
	  NEXTION_SEND(huart6, 2, charge_code, ecar.BMS.bms_error_code, myMessagex);
	  NEXTION_SEND(huart6, 0, voltage_err_code, ecar.CHARGER.voltage_error, myMessagex);
	  NEXTION_SEND(huart6, 0, current_err_code, ecar.CHARGER.current_error, myMessagex);
	  NEXTION_SEND(huart6, 0, temperature_err_code, ecar.CHARGER.temperature_error, myMessagex);

	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  //HAL_Delay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_rf */
/**
* @brief Function implementing the rf thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_rf */
void start_rf(void *argument)
{
  /* USER CODE BEGIN start_rf */
  /* Infinite loop */
  for(;;)
  {
	  //deneme();
	  if(prepare_data()){
		 HAL_UART_Transmit(&huart2, (uint8_t*)arr, strlen(arr), 1000);
		 memset(arr, 0, 300 * sizeof(char));
		 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		//HAL_Delay(1000);
	 }
	//HAL_Delay(50);
  }
  /* USER CODE END start_rf */
}

/* USER CODE BEGIN Header_start_can */
/**
* @brief Function implementing the can thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_can */
void start_can(void *argument)
{
  /* USER CODE BEGIN start_can */
	pTxHeader.StdId = 0x10;
	pTxHeader.DLC = 1;
	int check_charge_loop=0;
  /* Infinite loop */
  for(;;)
  {
	  uint8_t receive_flag=0;
	  //__________CHARGER___________

	  if(ecar.CHARGER.start_charge_state==(int)'I' || check_charge_loop<3){
		  if(ecar.CHARGER.start_charge_state ==(int)'S' && ecar.BMS.bms_error_code){
			 HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, (uint8_t*)41, &pTxMailbox);  //start charger
			 //WaitForMission(0x19, 0x10, 100, 41);
			 ecar.CHARGER.start_charge_state = 'I';
			 check_charge_loop=0;
		  }
		  else if((ecar.CHARGER.start_charge_state ==(int)'B' && ecar.BMS.bms_error_code!=1) && check_charge_loop<3 ){
			  //HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, (uint8_t*)40, &pTxMailbox);  //check bms
			  WaitForMission(0x19, 0x10, 100, 40);
			  check_charge_loop++;
		  }
		  else if(ecar.CHARGER.start_charge_state==(int)'C'){
			  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, (uint8_t*)82, &pTxMailbox);  //operation cancel
			  //WaitForMission(0x19, 0x10, 100, 41);
			  ecar.CHARGER.start_charge_state = 'I';
			  check_charge_loop=0;
		  }
		  if(check_charge_loop==3){
			  ecar.CHARGER.start_charge_state = 'I';
			  check_charge_loop=0;
		  }
	  }
	  //__________CHARGER___________

	  HAL_Delay(50);

	  if(!(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, RxData))){
	  switch (pRxHeader.StdId) {
	  	case 0x12:
	  		//for if 5.input is direction : => 0b0001 0000 = 16
	  		decimal_to_bin(RxData[0]);

	  		ecar.RELAY.stop_ = dest[0];
			ecar.RELAY.hazard_ = dest[1];
			ecar.RELAY.left_ = dest[2];
			ecar.RELAY.right_ = dest[3];
			ecar.RELAY.light_ = dest[4];
			ecar.RELAY.wiper_ = dest[5];
			ecar.RELAY.horn_ = dest[6];
			ecar.RELAY.direction_ = dest[7]==1?'+':'-';

	  	  	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	  	  	receive_flag=1;
	  	  	break;
	  	case 0x14:
			ecar.DRIVER.vehicle_speed = RxData[0];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x16:
			ecar.CHARGER.charge_watt = (RxData[0]*100)+RxData[1];
			ecar.CHARGER.ReadyToFlow = RxData[3];
			ecar.CHARGER.voltage_error = RxData[4];
			ecar.CHARGER.current_error = RxData[5];
			ecar.CHARGER.temperature_error = RxData[6];
			ecar.CHARGER.time = RxData[7]*60+RxData[8]; //second
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
	  	case 0x19:
	  		ecar.BMS.bms_error_code = RxData[0];
	  		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	  		receive_flag=1;
	  		break;
		case 0x20:
			ecar.BMS.battery_voltage[0]=RxData[0]*100+RxData[1];
			ecar.BMS.battery_voltage[1]=RxData[2]*100+RxData[3];
			ecar.BMS.battery_voltage[2]=RxData[4]*100+RxData[5];
			ecar.BMS.battery_voltage[3]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x21:
			ecar.BMS.battery_voltage[4]=RxData[0]*100+RxData[1];
			ecar.BMS.battery_voltage[5]=RxData[2]*100+RxData[3];
			ecar.BMS.battery_voltage[6]=RxData[4]*100+RxData[5];
			ecar.BMS.battery_voltage[7]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x22:
			ecar.BMS.battery_voltage[8]=RxData[0]*100+RxData[1];
			ecar.BMS.battery_voltage[9]=RxData[2]*100+RxData[3];
			ecar.BMS.battery_voltage[10]=RxData[4]*100+RxData[5];
			ecar.BMS.battery_voltage[11]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x23:
			ecar.BMS.battery_voltage[12]=RxData[0]*100+RxData[1];
			ecar.BMS.battery_voltage[13]=RxData[2]*100+RxData[3];
			ecar.BMS.battery_voltage[14]=RxData[4]*100+RxData[5];
			ecar.BMS.battery_voltage[15]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x24:
			ecar.BMS.battery_voltage[16]=RxData[0]*100+RxData[1];
			ecar.BMS.battery_voltage[17]=RxData[2]*100+RxData[3];
			ecar.BMS.battery_voltage[18]=RxData[4]*100+RxData[5];
			ecar.BMS.battery_voltage[19]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x25:
			ecar.BMS.battery_voltage[20]=RxData[0]*100+RxData[1];
			ecar.BMS.battery_voltage[21]=RxData[2]*100+RxData[3];
			ecar.BMS.battery_voltage[22]=RxData[4]*100+RxData[5];
			ecar.BMS.battery_voltage[23]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
		case 0x26:
			ecar.BMS.battery_temparature = RxData[0];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			receive_flag=1;
			break;
	  }
	  if(receive_flag){
		  uint8_t can_feedback[1];can_feedback[0]=pRxHeader.StdId;
		  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, can_feedback, &pTxMailbox);
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	  }
	  }
	}
  /* USER CODE END start_can */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the dac thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	lcd_init();
	lcd_reset();
	int counterr=0;
	for(int i=0;i<16;i++){
		lcd_send_cmd(0x80);
		lcd_send_string(counterr==0?"LOADING.  ":(counterr==1?"LOADING.. ":"LOADING..."));
		lcd_send_cmd(0xC0+i);
		lcd_send_string("#");
		HAL_Delay(100);
		counterr++;
		if(counterr>2){counterr=0;};
	}
	lcd_send_cmd(0x80);
	lcd_send_string("                ");
	HAL_Delay(200);
	lcd_send_cmd(0x80);
	lcd_send_string("COMPLETED!");
	HAL_Delay(300);
	lcd_send_cmd(0x80);
	lcd_send_string("                ");
	HAL_Delay(200);
	lcd_send_cmd(0x80);
	lcd_send_string("COMPLETED!");
	HAL_Delay(300);
	lcd_send_cmd(0x80);
	lcd_send_string("                ");
	HAL_Delay(200);
	lcd_send_cmd(0x80);
	lcd_send_string("COMPLETED!");
	HAL_Delay(300);
	lcd_send_cmd(0x80);
	lcd_send_string("                ");
	HAL_Delay(200);
	lcd_send_cmd(0x80);
	lcd_send_string("COMPLETED!");
	HAL_Delay(300);

	HAL_Delay(500);
	lcd_reset();
	HAL_Delay(100);
	sprintf(&M[0],"%s", "E");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x83);
	sprintf(&M[0],"%s", "L");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x84);
	sprintf(&M[0],"%s", "E");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x85);
	sprintf(&M[0],"%s", "C");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x86);
	sprintf(&M[0],"%s", "T");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x87);
	sprintf(&M[0],"%s", "R");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x88);
	sprintf(&M[0],"%s", "I");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x89);
	sprintf(&M[0],"%s", "C");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x8A);
	sprintf(&M[0],"%s", "A");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x8B);
	sprintf(&M[0],"%s", "R");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0x8C);
	HAL_Delay(100);
	lcd_send_cmd(0xC0);
	sprintf(&M[0],"%s", "T");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC2);
	sprintf(&M[0],"%s", "H");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC3);
	sprintf(&M[0],"%s", "U");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC4);
	sprintf(&M[0],"%s", "N");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC5);
	sprintf(&M[0],"%s", "D");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC6);
	sprintf(&M[0],"%s", "E");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC7);
	sprintf(&M[0],"%s", "R");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC8);
	sprintf(&M[0],"%s", "V");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xC9);
	sprintf(&M[0],"%s", "O");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xCA);
	sprintf(&M[0],"%s", "L");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xCB);
	sprintf(&M[0],"%s", "T");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xCC);
	sprintf(&M[0],"%s", "T;");HAL_Delay(100);lcd_send_string(&M[0]);lcd_send_cmd(0xCD);

	HAL_Delay(500);
	//int state_of_tim=0;
	int aim_point = 30;
	int step_number = 10;
	//int countrr=0;
	TIM2->CCR3 = 0;
  /* Infinite loop */
  for(;;)
  {

	  SEND_DATA_TO_LCD();
	  temp_of_lcd = ecar.BMS.battery_temparature;
	  //if(temp_of_lcd<aim_point && state_of_tim){HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);state_of_tim = 0;fan_level=0;}
	  if(temp_of_lcd>aim_point){//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);state_of_tim = 1;
	  fan_level=0;}
	  if((temp_of_lcd>aim_point && temp_of_lcd<=aim_point + step_number)){//TIM2->CCR3 = 25;
	  fan_level=25;}
	  if((temp_of_lcd>aim_point + step_number && temp_of_lcd<=aim_point + 2*step_number)){//TIM2->CCR3 = 50;
	  fan_level=50;}
	  if((temp_of_lcd>aim_point + 2*step_number && temp_of_lcd<=aim_point + 3*step_number)){//TIM2->CCR3 = 75;
	  fan_level=75;}
	  if((temp_of_lcd>aim_point + 3*step_number && temp_of_lcd<=aim_point + 4*step_number)){//TIM2->CCR3 = 100;
	  fan_level=100;}
	  CalculateMaxBattery();
	  CalculateMinBattery();
	  HAL_Delay(1000);
  }
  /* USER CODE END StartTask04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
 /* HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  HAL_Delay(200);*/
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
