/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdbool.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
    float f_val;
    uint8_t bytes[4];
} FloatConverter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIN_TABLE_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/*pc*/
uint8_t RxChar;
char RxBuffer[32];
int RxIndex=0;
volatile bool CanTxFlag = false;
volatile bool CanRxFlag = false;
uint32_t CanTxIde;
uint32_t CanTxId;
uint32_t CanTxDlc;
uint32_t CanRxIde;
uint32_t CanRxId;
uint32_t CanRxDlc;

/*can*/
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t CanTxData[8] = {0};
uint32_t id;
uint8_t CanRxData[8];
FDCAN_RxHeaderTypeDef RxHeader;

/*Motor*/
int MotorId = 127;//0x7F
float TargetTorque = 0;
float Angle = 0;
float Velocity = 0;
float Torque = 0;
float Temp = 0;

/*PC*/
char tx_buffer[256];

/*その他*/
volatile bool state = false;
volatile int Timer_ms = 0;
float Freq = 4;
int McuId = 127;
int Hz = 1000;

float SinTable[SIN_TABLE_SIZE];
float Phase1 = 0.0f;
float Phase2 = 0.0f;
float Phase3 = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void){
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x02000000;
	sFilterConfig.FilterID2 = 0x1F000000;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
		Error_Handler();
	}
    /* Configure global filter to reject all non-matching frames */
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK){
        Error_Handler();
    }
	if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
		Error_Handler();
	}
	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		Error_Handler();
	}
	TxHeader.Identifier = 0x123;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	uint8_t RxData[8];
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
		if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,&RxHeader, RxData)!= HAL_OK){
			Error_Handler();
		}
		CanRxIde = RxHeader.IdType;
		CanRxId = RxHeader.Identifier;
		CanRxDlc = RxHeader.DataLength;
		CanRxData[0] = RxData[0];                                                    // Data
		CanRxData[1] = RxData[1];
		CanRxData[2] = RxData[2];
		CanRxData[3] = RxData[3];
		CanRxData[4] = RxData[4];
		CanRxData[5] = RxData[5];
		CanRxData[6] = RxData[6];
		CanRxData[7] = RxData[7];
		CanRxFlag = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1){
		if(RxChar == 's'){//startのs
			state = 1;
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
				TxHeader.Identifier = 0x3<<24 | McuId<<8 | MotorId;
				for(uint8_t i = 0; i<8; i++)CanTxData[i] = 0;
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) == HAL_OK) {
					//送信した時に行いたい処理
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				}
			}
		}else if(RxChar == 'e'){//endのe
			state = 0;
			Timer_ms=0;
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
				TxHeader.Identifier = 0x4<<24 | McuId<<8 | MotorId;
				for(uint8_t i = 0; i<8; i++)CanTxData[i] = 0;
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) == HAL_OK) {
					//送信した時に行いたい処理
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				}
			}
		}
		HAL_UART_Receive_IT(&huart1, &RxChar,1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim16){//1msごとに呼び出される
		if(state){
			CanTxFlag = true;
			Timer_ms+=1000/Hz;//1msをプラス
		}else{
		}
	}
}

//uint32_t parseHex(char* str, int len){
//	uint32_t val = 0;
//	for(int i = 0;i<len;i++){
//		char c = str[i];
//		uint8_t v = 0;
//		if(c >= '0' && c<= '9') v = c - '0';//数字に変換
//		else if(c >= 'A' && c <= 'F') v = c - 'A' + 10;//大文字の16進数も10進数に変換
//		else if(c >= 'a' && c <= 'f') v = c - 'a' + 10;//小文字の16進数も10進数に変換
//		val = (val << 4) | v;
//	}
//	return val;
//}
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  FDCAN_Config();
  HAL_UART_Receive_IT(&huart1, &RxChar, 1);
  HAL_TIM_Base_Start_IT(&htim16);

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
	  TxHeader.Identifier = 0x3<<24 | McuId<<8 | MotorId;
	  for(uint8_t i = 0; i<8; i++)CanTxData[i] = 0;
	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) == HAL_OK) {
		  //送信した時に行いたい処理
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
  }


  for(int i = 0; i<SIN_TABLE_SIZE; i++){
	  SinTable[i] = sinf(2.0f*(float)M_PI*(float)i/(float)SIN_TABLE_SIZE);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(CanRxFlag){
		  static uint8_t sendToPc[12];
		  sendToPc[0] = Timer_ms & 0x000000ff;
		  sendToPc[1] = (Timer_ms & 0x0000ff00) >> 8;
		  sendToPc[2] = (Timer_ms & 0x00ff0000) >> 16;
		  sendToPc[3] = (Timer_ms & 0xff000000) >> 24;
		  sendToPc[4] = CanRxData[0];
		  sendToPc[5] = CanRxData[1];
		  sendToPc[6] = CanRxData[2];
		  sendToPc[7] = CanRxData[3];
		  sendToPc[8] = CanRxData[4];
		  sendToPc[9] = CanRxData[5];
		  sendToPc[10] = CanRxData[6];
		  sendToPc[11] = CanRxData[7];
		  HAL_UART_Transmit(&huart1, sendToPc , 12, 100);//64bit + 32bit = 96bit = 12byte
//		  if(huart1.gState != HAL_UART_STATE_BUSY_TX){
//			  HAL_UART_Transmit_DMA(&huart1, sendToPc, 12);
//		  }
		  CanRxFlag=false;
	  }

	  if(CanTxFlag){
		  //目標トルクをマルチサイン波にしたければこちら
		  float delta_phase1 = Freq * 0.001f;
		  float delta_phase2 = 3.4f * Freq * 0.001f;
		  float delta_phase3 = 7.4f * Freq * 0.001f;

		  Phase1 += delta_phase1;
		  Phase2 += delta_phase2;
		  Phase3 += delta_phase3;

		  if(Phase1 >= 1.0f) Phase1 -= 1.0f;
		  if(Phase2 >= 1.0f) Phase2 -= 1.0f;
		  if(Phase3 >= 1.0f) Phase3 -= 1.0f;

		  int idx1 = (int)(Phase1 * SIN_TABLE_SIZE);
		  int idx2 = (int)(Phase2 * SIN_TABLE_SIZE);
		  int idx3 = (int)(Phase3 * SIN_TABLE_SIZE);

		  if(idx1 >= SIN_TABLE_SIZE) idx1 = 0;
		  if(idx2 >= SIN_TABLE_SIZE) idx2 = 0;
		  if(idx3 >= SIN_TABLE_SIZE) idx3 = 0;

		  TargetTorque = 1.0f*(SinTable[idx1] + 0.6f*SinTable[idx2] + 0.3f*SinTable[idx3]);

		  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
			  int targetTorque_int = (int)((TargetTorque + 5.5) * 65535 / 11);
			  TxHeader.Identifier = 0x1<<24 | targetTorque_int<<8 | MotorId;
			  for(uint8_t i = 0; i<8; i++)CanTxData[i] = 0;
			  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) == HAL_OK) {
				  //送信した時に行いたい処理
				  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			  }
		  }
		  TargetTorque = 1.0f*(SinTable[idx1] + 0.6f*SinTable[idx2] + 0.3f*SinTable[idx3]);
		  float targetTorque = 0;
		  CanTxFlag = false;

		  //目標スピードを階段状にしたければこちら
		  /*
		  float targetSpeed;
		  float kp = 0.0;
		  float kd = 0.1;
		  if(Timer_ms < 6000){
			  targetSpeed = 2.0f * (float)floor((float)Timer_ms * 0.001f);
		  }else if(Timer_ms < 10000){
			  targetSpeed = 2.0f * (float)floor(11.0f - (float)Timer_ms*0.001f);
		  }else{
			  targetSpeed = 0;
		  }
		  uint16_t targetSpeed_int = (uint16_t)((targetSpeed + 50.0f) / 100.0f * 65535.0f);
		  uint16_t targetTorque_int = 32768;
		  uint16_t targetAngle_int = 32768;
		  uint16_t kp_int = (uint16_t)(kp*65535.0f/500.0f);
		  uint16_t kd_int = (uint16_t)(kd*65535.0f/5.0f);
		  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
			  TxHeader.Identifier = 0x1<<24 | targetTorque_int<<8 | MotorId;
			  CanTxData[0] = (targetAngle_int & 0xff00)>>8;
			  CanTxData[1] = targetAngle_int & 0x00ff;
			  CanTxData[2] = (targetSpeed_int & 0xff00)>>8;
			  CanTxData[3] = targetSpeed_int & 0x00ff;
			  CanTxData[5] = (kp_int & 0xff00)>>8;
			  CanTxData[4] = kp_int & 0x00ff;
			  CanTxData[6] = (kd_int & 0xff00)>>8;
			  CanTxData[7] = kd_int & 0x00ff;
			  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) == HAL_OK) {
				  //送信した時に行いたい処理
				  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			  }
		  }
		  CanTxFlag = false;
		  */
	  }

	  // Bus-Offを検知したら、FDCANを一旦停止して再スタート（エラー解除）
	  if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0) {
		  HAL_FDCAN_Stop(&hfdcan1);
		  HAL_FDCAN_Start(&hfdcan1);
	  }
//	  HAL_Delay(1);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 8;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x00402D41;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 12-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
