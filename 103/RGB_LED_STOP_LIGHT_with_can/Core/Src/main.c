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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch3;

/* USER CODE BEGIN PV */



  CAN_TxHeaderTypeDef pTxHeader;
  CAN_RxHeaderTypeDef pRxHeader;
  CAN_FilterTypeDef sFilterConfig;
  uint32_t pTxMailbox;
  uint8_t RxData[2];
  uint8_t Txdata=10;
  uint8_t CanReceive=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_LED 74
#define USE_BRIGHTNESS 0
#define l 15
#define RIGHT_SIGNAL MAX_LED-l
#define LEFT_SIGNAL 0
#define STOP_SIGNAL l
#define PI 3.14159265
int color;int adc_val;int adc_val2;


uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

int datasentflag=0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}


void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}

void right_signal(){
	for(int i=RIGHT_SIGNAL;i<MAX_LED;i++){
			  Set_LED(i, 250, 50, 0);
//			  for(int j=RIGHT_SIGNAL;j<MAX_LED;j++){
//				  if(j==i||j==i-1 || j==i-2){
//				  }else{
//					 // Set_LED(j, 0, 0, 0);
//				  }
//			  }
			  WS2812_Send();
			  HAL_Delay(20);
		  }
}

void stop_signal(){
	for(int i=14;i<74-15;i++){
		Set_LED(i, 250, 0, 0);
	}
	WS2812_Send();
	HAL_Delay(100);
}

void left_signal(){
	for(int i=14;i>=0;i--){
		Set_LED(i, 250, 50, 0);
		for(int j=14;j>=0;j--){
		if(j==i || j==i+1 || j==i+2){
		}else{
		    //Set_LED(j, 0, 0, 0);
		}
		}
		WS2812_Send();
		HAL_Delay(20);
	}
}

void stop_w_left(){
	for(int i=STOP_SIGNAL;i<MAX_LED-l;i++){
			Set_LED(i, 250, 0, 0);
	}
	for(int i=14;i>=0;i--){
		Set_LED(i, 250, 120, 0);
//		for(int j=14;j>=0;j--){
//		if(j==i || j==i+1 || j==i+2){
//		}else{
//			 Set_LED(j, 0, 0, 0);
//		}
//		}
		WS2812_Send();
		HAL_Delay(25);
	}
}
void stop_w_right(){
	for(int i=STOP_SIGNAL;i<MAX_LED-l;i++){
				Set_LED(i, 250, 0, 0);
		}
		for(int i=74-15;i<74;i++){
		Set_LED(i, 250, 120, 0);
//			for(int j=74-15;j<74;j++){
//			if(j==i || j==i-1 ||j==i-2){
//			}else{
//				 Set_LED(j, 0, 0, 0);
//			}
//			}
			WS2812_Send();
			HAL_Delay(25);
		}
}


void all_together(){
	for(int i=STOP_SIGNAL;i<MAX_LED-l;i++){
				Set_LED(i, 250, 0, 0);
		}
		for(int i=MAX_LED-l-l-1;i>=LEFT_SIGNAL;i--){
			Set_LED(i, 250, 50, 0);
			for(int j=MAX_LED-l-l;j>LEFT_SIGNAL;j--){
			if(j==i || j==i+1){
			}else{
				 Set_LED(j, 0, 0, 0);
			}
			}
			for(int i=RIGHT_SIGNAL;i<MAX_LED;i++){
						  Set_LED(i, 250, 50, 0);
						  for(int j=RIGHT_SIGNAL;j<MAX_LED;j++){
							  if(j==i||j==i-1){
							  }else{
								  Set_LED(j, 0, 0, 0);
							  }
				}
				WS2812_Send();
				HAL_Delay(15);
			}

		}
}

void all(){
	int k=0;
	int reset_p=59;
	int indexes=2;
	for(int i=STOP_SIGNAL;i<MAX_LED-l;i++){
			Set_LED(i, 250, 0, 0);
	}
	for(int i=14;i>=0;i--){
		Set_LED(i, 250, 50, 0);
		for(int j=14;j>=0;j--){

			if(j==i || j==i+1 || j==i+2 ){}
				else{
					Set_LED(j, 0, 0, 0);
				}
		}

		int led_nu = (i*4)+indexes+1;
		Set_LED(led_nu, 250, 50, 0);
		k++;
		indexes+=5;
		if(k>3){
			Set_LED(reset_p, 0, 0, 0);
			reset_p++;
		}
		if(i==0){
			Set_LED(73, 250, 50, 0);
		}

		WS2812_Send();
		Set_LED(0, 0, 0, 0);
		HAL_Delay(15);
	}
}

void left_right(){
	int k=0;
	int reset_p=59;
	int indexes=2;

	for(int i=14;i>=0;i--){
		Set_LED(i, 250, 50, 0);
		for(int j=14;j>=0;j--){

			if(j==i || j==i+1 || j==i+2 ){}
				else{
					Set_LED(j, 0, 0, 0);
				}
		}

		int led_nu = (i*4)+indexes+1;
		Set_LED(led_nu, 250, 50, 0);
		k++;
		indexes+=5;
		if(k>3){
			Set_LED(reset_p, 0, 0, 0);
			reset_p++;
		}
		if(i==0){
			Set_LED(73, 250, 50, 0);
		}

		WS2812_Send();
		Set_LED(0, 0, 0, 0);
		HAL_Delay(20);
	}
}
int value=0;
void opening_sim(){
	int k=0;
	int reset_p=37;
	int adım_say=1;
	while(1){
	for(int i=36;i>=value;i--){
		Set_LED(i, 250, 0, 0);
		for(int j=36;j>=value+2;j--){
			if(j==i || j==i+1 || j==i+2){}
			else{
				Set_LED(j, 0, 0, 0);
			}
		}
	Set_LED(i+adım_say, 250, 0, 0);
	adım_say+=2;
	k++;
	if(k>3){
		Set_LED(reset_p, 0, 0, 0);
		reset_p++;
	}
	WS2812_Send();
	HAL_Delay(5);
	}
	k=0;
	reset_p=37;
	adım_say=1;
	value+=2;

	if(value>36){break;}
	}
}

void to_visibilty(){

	int r=250;
	while(1){
		for(int i=0;i<75;i++){
			Set_LED(i, r, 0,0);
		}
		HAL_Delay(15);
		//Set_Brightness(45);
		WS2812_Send();
		r--;

		if(r==0){Reset_LED();WS2812_Send();HAL_Delay(200);break;}
	}
}
void all_reset(){
	Reset_LED();
}


#define stop stop_signal()
#define left left_signal()
#define right right_signal()
#define left_right left_right()
#define stop_w_left stop_w_left()
#define stop_w_right stop_w_right()
#define stop_left_right all()
#define all_reset all_reset()


int state=0;

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

void All_Set_LED (int r,int b,int g)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = r;
		LED_Data[i][2] = g;
		LED_Data[i][3] = b;
	}
}

int dest[8];
void decimal_to_bin(uint8_t number){ // EX for 255 = [1][1][1][1][1][1][1][1] binary code
    for(int i=0;i<8;i++){
        dest[i] = number%2;
        number/=2;
    }
}
uint8_t can_flag=1;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  //can comm.

  HAL_CAN_Start(&hcan);   //start can communication

  // HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // enable interrupt
  pTxHeader.DLC = 1;
  pTxHeader.IDE = CAN_ID_STD;
  pTxHeader.RTR = CAN_RTR_DATA;
  pTxHeader.StdId = 0x13;

  //set filter parameters
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //just communicate with relay board
  sFilterConfig.FilterIdHigh = 0x12 << 5;
  sFilterConfig.FilterIdLow = 0x12 << 5;
  sFilterConfig.FilterMaskIdHigh = 0xFF << 5;
  sFilterConfig.FilterMaskIdLow = 0xFF<< 5;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;


  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);


  Reset_LED();
  WS2812_Send();
  Set_Brightness(45);
  opening_sim();
  to_visibilty();
  Reset_LED();
  WS2812_Send();
  HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

   HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRxHeader, RxData);

   //INPUTS: 1-stop; 2-hazard; 3-left; 4-right; 5-light; 6-wiper; 7-horn; 8-switch;
   //OUTPUTS : 1-?; 2-left; 3-?; 4-light; 5-wiper; 6-horn; 7-? 8-right;

   if(pRxHeader.StdId == 0x12){   //relay id
	  if(RxData[0]==0){
		   Reset_LED();
		   WS2812_Send();
		   continue;
	  }
	  decimal_to_bin(RxData[0]);
	  //if all is 1 running this code
	  if((dest[0] && dest[1] && dest[2] && dest[3]) || (dest[0] && dest[1] && dest[2]) || (dest[0] && dest[1] && dest[3]) || (dest[0] && dest[1]) || (dest[0] && dest[2] && dest[3])){ // stop
		   stop_left_right;
		   all_reset;
		   continue;
	  }
	  else if(dest[0] && dest[2]){ // stop-left
		   stop_w_left;
		   all_reset;
		   continue;
	  }
	  else if(dest[0] && dest[3]){ // stop-right
		   stop_w_right;
		   all_reset;
		   continue;
	  }
	  else if(dest[0]){ // stop
		   stop;
		   all_reset;
		   continue;
	  }
	  else if(dest[2]){ // left
		   left;
		   all_reset;
		   continue;
	  }
	  else if(dest[3]){ // right
		   right;
		   all_reset;
		   continue;
	  }
	  else if(dest[1]){ // hazard
		   left_right;
		   all_reset;
		   continue;
	  }
	}
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 72-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 72-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
