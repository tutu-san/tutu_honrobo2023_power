/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "power_ctrl.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef void (*void_void_func)(void);
typedef float (*float_void_func)(void);
typedef uint32_t (*uint32_void_func)(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BATT_VOLTGE_PIN GPIOA,GPIO_PIN_0
#define OUTPUT_VOLTGE_PIN GPIOA,GPIO_PIN_1
#define OUTPUT_CURRENT_PIN GPIOA,GPIO_PIN_2
#define CELL_CONFIG_PIN GPIOA,GPIO_PIN_3
#define SD_SIGN_PIN GPIOA,GPIO_PIN_4
#define CURRENT_SIGN_PIN GPIOA,GPIO_PIN_5
#define CURRENT_RESET_PIN GPIOA,GPIO_PIN_6
#define DISCHARGE_PIN GPIOA,GPIO_PIN_7
#define POWER_SD_PIN GPIOA,GPIO_PIN_8
#define LED_G_PIN GPIOA,GPIO_PIN_15
#define LED_B_PIN GPIOB,GPIO_PIN_0
#define LED_R_PIN GPIOB,GPIO_PIN_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

void_void_func power_func[] = { POWER_OFF, POWER_ON, CURRENT_RESET }; //関数ポインタを配列に入れる。改造したいけどよくわからなかったら、switch()に改修するので教えて。
uint8_t SW_state = 0; //自基板の停止ボタンの状態を記憶する変数

//CAN受信がらみ
uint32_t id;
uint32_t dlc;
uint8_t data[8];

//ADC
uint8_t ADC_buf[4] = { 0, 0, 0, 0 }; //PA0,1,2,3

//リレーの状態
enum {
	POWER_OUTPUT_HIGH, POWER_OUTPUT_LOW
};

//自基板の停止ボタンの状態
enum {
	REMOTE_OFF, SW_OFF, ALL_ON
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
uint32_t CAN_write(uint32_t StdID, uint32_t RTR, uint32_t DLC, uint8_t Data[]); //CAN送信をする関数
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
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

	uint32_t fId1 = 0x000 << 21;
	// フィルターID1
	uint32_t fId2 = 0x001 << 21;  // フィルターID2

	CAN_FilterTypeDef filter;
	filter.FilterIdHigh = 0;            // フィルターID1の上位16ビット
	filter.FilterIdLow = 0;                  // フィルターID1の下位16ビット
	filter.FilterMaskIdHigh = 0;            // フィルターID2の上位16ビット
	filter.FilterMaskIdLow = 0;                  // フィルターID2の下位16ビット
	filter.FilterScale = CAN_FILTERSCALE_32BIT; // フィルタースケール
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // フィルターに割り当てるFIFO
	filter.FilterBank = 0; // フィルターバンクNo
	filter.FilterMode = CAN_FILTERMODE_IDMASK; //フィルターモード
	filter.SlaveStartFilterBank = 14; //スレーブCANの開始フィルターバンクNo
	filter.FilterActivation = ENABLE; //フィルター無効／有効
	HAL_CAN_ConfigFilter(&hcan, &filter); //設定を確定
	HAL_CAN_Start(&hcan); //CAN開始
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //受信割り込みを開始

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET); //過電流検知リセット
	POWER_ON();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (HAL_GPIO_ReadPin(SD_SIGN_PIN) == 0) {        //リレーがOFFしたとき
			if (GET_GPIO_STATE(POWER_SD_PIN) == 1) { //自分の遠隔停止がONなら相手のリレーをOFFする(こちらのSWがOFFされた)
				if (SW_state != SW_OFF) {      //まだ相手に停止指令を送っていなければ送る
					uint8_t data[] = { POWER_OUTPUT_LOW };
					CAN_write(0x000, CAN_RTR_DATA, 0x1, data);
					SET_LED_R();

				}
				SW_state = SW_OFF;
			} else {        //遠隔停止がOFFなら相手の遠隔停止によって停止(何も送らずに待機)
				SW_state = REMOTE_OFF;
				SET_LED_Y();
			}
		} else {
			//リレーがONのとき
			if (SW_state != ALL_ON) {    //自分の停止SWが解除されたのとき、まだ相手にON指令を送っていなければ送る
				uint8_t data[] = { POWER_OUTPUT_HIGH };
				CAN_write(0x000, CAN_RTR_DATA, 0x1, data);
			}
			SW_state = ALL_ON;
			SET_LED_B();

		}

		/*ADCデバッグ用。消しても問題なし*/
		/*
		 char buf[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		 0 };
		 snprintf(buf, sizeof(buf), "%3d", (int) (ADC_buf[2]));
		 HAL_UART_Transmit(&huart1, (uint8_t*) buf, sizeof(buf), 1000);
		 char mozi[] = "\r\n";
		 HAL_UART_Transmit(&huart1, (uint8_t*) mozi, sizeof(mozi), 1000);
		 */

		char SW_str[20];
		if (SW_state == SW_OFF) {
			snprintf(SW_str, sizeof("SW_OFF"), "%s", "SW_OFF");
			HAL_UART_Transmit(&huart1, (uint8_t*) SW_str, sizeof("SW_OFF"),
					100);
		} else if (SW_state == REMOTE_OFF) {
			snprintf(SW_str, sizeof("REMOTE_OFF"), "%s", "REMOTE_OFF");
			HAL_UART_Transmit(&huart1, (uint8_t*) SW_str, sizeof("REMOTE_OFF"),
					100);
		} else if (SW_state == ALL_ON) {
			snprintf(SW_str, sizeof("ALL_ON") + 1, "%s", "ALL_ON");
			HAL_UART_Transmit(&huart1, (uint8_t*) SW_str, sizeof("ALL_ON"),
					100);
		}
		char mozi[] = "\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) mozi, sizeof(mozi), 1000);

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUTPUT_current_reset_Pin|OUTPUT_POWER_SD_Pin|OUTPUT_LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_LED_B_Pin|OUTPUT_LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           SD_SIGN_Pin CURRENT_SIGN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |SD_SIGN_Pin|CURRENT_SIGN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_current_reset_Pin OUTPUT_POWER_SD_Pin OUTPUT_LED_G_Pin */
  GPIO_InitStruct.Pin = OUTPUT_current_reset_Pin|OUTPUT_POWER_SD_Pin|OUTPUT_LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_LED_B_Pin OUTPUT_LED_R_Pin */
  GPIO_InitStruct.Pin = OUTPUT_LED_B_Pin|OUTPUT_LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//CAN送信
//CAN送信
uint32_t CAN_write(uint32_t StdID, uint32_t RTR, uint32_t DLC, uint8_t Data[]) {
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
		CAN_TxHeaderTypeDef TxHeader; //CANのデータを突っ込む構造体
		uint32_t TxMalibox; //送信に使用したメールボックスの番号を突っ込む

		//IDを設定
		//範囲0x0000~0x7ff
		TxHeader.StdId = StdID;

		//フレームの種類を設定
		//データフレーム:CAN_RTR_DATA
		//リモートフレーム:CAN_RTR_REMOTE
		TxHeader.RTR = RTR;

		//標準IDor拡張ID
		//標準ID:CAN_ID_STD
		//拡張ID:CAN_ID_EXT
		TxHeader.IDE = CAN_ID_STD;

		//送信したデータ長
		//範囲0x0~0x8
		TxHeader.DLC = DLC;

		//タイムスタンプを送るかどうかの設定
		//無効でよいかと
		TxHeader.TransmitGlobalTime = DISABLE;

		//送信
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, Data, &TxMalibox);

		return TxMalibox;
	} else {
		return 4;
	}
}

//CAN受信割り込み
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];		//データを入れる配列

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {//受信
		id = (RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId; // IDをコピー
		dlc = RxHeader.DLC; // DLC
		/*ここに処理を書く*/
		power_func[RxData[0]](); //配列に入れた関数ポインタを通じて関数が呼び出される

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
	SET_LED_W();
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
