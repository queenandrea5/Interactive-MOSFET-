/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
#include "logger.h"
#include "logger_lpuart.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
UART_HandleTypeDef huart1; // ECRAN (USART1 PB6/PB7)
UART_HandleTypeDef hlpuart1;

typedef enum {
	MODE_DISPLAY = 0,   // affichage normal des valeurs
	MODE_ASK_VTH,       // l’écran demande "Entrez Vth"
	MODE_WAIT_VTH       // on attend l’entrée clavier
} ui_mode_t;

#define VTH      1800
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUF_SZ 128
#define MSG_SIZE 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
ui_mode_t ui_mode = MODE_DISPLAY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t  rx_byte = 0;
static char     rxbuf[RXBUF_SZ];
static volatile uint16_t rxlen = 0;
static volatile uint8_t  line_ready = 0;
volatile uint8_t intro_done = 0;
uint8_t rxChar;           // un caractère
char rxBuffer[100];       // message complet
uint8_t indexRx = 0;
volatile uint8_t messageReady = 0;
uint8_t  messageStarted = 0;
volatile uint8_t ackTxDone = 1;

static uint32_t pv1 = 0, pv2 = 0, pv3 = 0,  pv4 = 0; // dernières valeurs (mV)
static uint32_t last = 0;
char msg[MSG_SIZE];

static void ep_show_ask_vth(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &hlpuart1) {
		if (!line_ready) {
			if (rx_byte == '\n') {                 // fin de ligne
				if (rxlen < RXBUF_SZ) rxbuf[rxlen] = 0;
				line_ready = 1;
			} else if (rx_byte != '\r') {
				if (rxlen < RXBUF_SZ - 1) rxbuf[rxlen++] = (char)rx_byte;
				else rxlen = 0; // overflow -> reset
			}
		}
		HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
	}


	else if(huart->Instance == USART1)
	{
		// Ignorer octets nuls ou parasites
		if (rxChar == '\0') {
			HAL_UART_Receive_IT(&huart1, &rxChar, 1);
			return;
		}

		// Détection du début de message
		if (rxChar == '#') {
			indexRx = 0;
			messageStarted = 1;  // On a vu le marqueur
		}
		else if (messageStarted) {
			// On commence à enregistrer seulement après le '#'
			if (rxChar == '\n') {   // Fin de message
				if (indexRx > 0) {
					rxBuffer[indexRx] = '\0';
					messageReady = 1;
				}
				messageStarted = 0; // On attend un nouveau '#'
				indexRx = 0;
			}
			else if (rxChar != '\r') {
				if (indexRx < sizeof(rxBuffer) - 1)
					rxBuffer[indexRx++] = rxChar;
				else
					indexRx = 0; // Sécurité : reset en cas de débordement
			}
		}

		// Relancer la réception du prochain octet
		HAL_UART_Receive_IT(&huart1, &rxChar, 1);
	}
}




// Trame: A5 | Len(2) | Cmd | payload | CC 33 C3 3C | XOR (sur tout sauf XOR lui-même)
static HAL_StatusTypeDef EP_SendFrame(uint8_t cmd, const uint8_t *payload, uint16_t plen)
{
	const uint8_t HEAD = 0xA5;
	const uint8_t TAIL[4] = {0xCC,0x33,0xC3,0x3C};
	uint16_t total = 1 + 2 + 1 + plen + 4 + 1;

	uint8_t buf[1 + 2 + 1 + 1024 + 4 + 1];
	uint16_t i=0;
	buf[i++]=HEAD;
	buf[i++]=(uint8_t)(total>>8);
	buf[i++]=(uint8_t)(total&0xFF);
	buf[i++]=cmd;
	if(plen && payload){ memcpy(&buf[i], payload, plen); i+=plen; }
	memcpy(&buf[i], TAIL, 4); i+=4;

	uint8_t x=0; for(uint16_t k=0;k<i;k++) x ^= buf[k];
	buf[i++]=x;

	return HAL_UART_Transmit(&hlpuart1, buf, i, 1000);
}

static void EP_Handshake(void){ EP_SendFrame(0x00,NULL,0); }
static void EP_Refresh (void){ EP_SendFrame(0x0A,NULL,0); }
static void EP_Clear   (void){ EP_SendFrame(0x2E,NULL,0); }
static void EP_SetPalette(uint8_t fg,uint8_t bg){ uint8_t p[2]={fg,bg}; EP_SendFrame(0x10,p,2); }
static void EP_SetFont(uint8_t idx){ uint8_t p[1]={idx}; EP_SendFrame(0x1E,p,1); } // 1:32,2:48,3:64




static void EP_DrawString(uint16_t x,uint16_t y,const char *s){
	uint16_t slen = (uint16_t)strlen(s)+1;
	uint8_t payload[4+1024];
	payload[0]=x>>8; payload[1]=x&0xFF;
	payload[2]=y>>8; payload[3]=y&0xFF;
	memcpy(&payload[4], s, slen);
	EP_SendFrame(0x30, payload, 4+slen);
}


// Boot "soft" : double handshake + clear + refresh
static void EP_BootSoft(void)
{
	EP_Handshake(); HAL_Delay(100);
	EP_Handshake(); HAL_Delay(100);
	EP_SetPalette(0x00,0x03); // noir/blanc
	EP_Clear();
	EP_SetFont(2);            // 48 points
	EP_Refresh();
}



static uint8_t parse_values(const char *s, uint32_t *v1, uint32_t *v2, uint32_t *v3, uint32_t *hid){
	if(s[0] != '#') return 0;
	const char *p1 = strstr(s,"VG="), *p2 = strstr(s,"VS="), *p3 = strstr(s,"VD="), *p4 = strstr(s,"Id=");
	if(!p1 || !p2 || !p3 || !p4) return 0;
	*v1 = strtoul(p1+3, NULL, 10);
	*v2 = strtoul(p2+3, NULL, 10);
	*v3 = strtoul(p3+3, NULL, 10);
	*hid = strtoul(p4+3, NULL, 10);
	return 1;
}

static void ep_show_intro(void){
	EP_Clear();
	EP_SetFont(2); // 48 pts

	EP_DrawString(100, 120, "CONFIGURATION MOSFET");
	EP_DrawString(100, 200, "Type: NMOS pedagogique");
	EP_DrawString(100, 280, "Seuil (Vth) = 1.8 V");
	EP_DrawString(100, 360, "Entrees: VG - VS - VD");
	EP_DrawString(100, 440, "Sortie: ID (mA)");

	EP_Refresh();
}

static void ep_show_values(uint32_t v1, uint32_t v2, uint32_t v3, uint32_t hid){
	/*char l1[40], l2[40], l3[40], l4[40];
  snprintf(l1,sizeof(l1),"VG = %lu mV", (unsigned long)v1);
  snprintf(l2,sizeof(l2),"VS = %lu mV", (unsigned long)v2);
  snprintf(l3,sizeof(l3),"VD = %lu  mV", (unsigned long)v3);
  snprintf(l4,sizeof(l4),"Id = %lu mA", (unsigned long)hid);
  EP_Clear();
  EP_SetFont(2);                   // 48 points
  EP_DrawString(30, 160, l1);     // 800x600
  EP_DrawString(30, 240, l2);
  EP_DrawString(30, 320, l3);
  EP_DrawString(30, 400, l4);
  EP_Refresh();*/

	EP_Clear();
	EP_SetFont(2);  // police 48 pts

	// Cadre supérieur
	EP_DrawString(20, 120,  "+------------------------------+");
	EP_DrawString(20, 160,  "| Parameter       |      Values      ");
	EP_DrawString(20, 200,  "+-----------+------------------+");

	char line[64];

	// Ligne VG
	snprintf(line, sizeof(line), "|   VG                       |   %4lu mV       ", (unsigned long)v1);
	EP_DrawString(20, 240, line);

	// Ligne VS
	snprintf(line, sizeof(line), "|   VS                       |   %4lu mV       ", (unsigned long)v2);
	EP_DrawString(20, 280, line);

	// Ligne VD
	snprintf(line, sizeof(line), "|   VD                       |   %4lu mV       ", (unsigned long)v3);
	EP_DrawString(20, 320, line);

	// VGS (calcul local)
	uint32_t vgs = (v1 > v2 ? v1 - v2 : 0);
	snprintf(line, sizeof(line), "|   VGS                    |   %4lu mV       ", (unsigned long)vgs);
	EP_DrawString(20, 360, line);

	// VDS
	uint32_t vds = (v3 > v2 ? v3 - v2 : 0);
	snprintf(line, sizeof(line), "|   VDS                    |   %4lu mV       ", (unsigned long)vds);
	EP_DrawString(20, 400, line);

	// ID
	snprintf(line, sizeof(line), "|   Id                       |   %4lu mA       ", (unsigned long)hid);
	EP_DrawString(20, 440, line);

	// Bas du tableau
	EP_DrawString(20, 480, "+------------------------------+");

	EP_Refresh();
}


static void ep_show_ask_vth(void){
	EP_Clear();
	EP_SetFont(2);
	EP_DrawString(50, 200, "Entrer nouvelle valeur de Vth");
	EP_DrawString(50, 280, "en mV puis pressez ENTREE");
	EP_Refresh();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1) {
		__HAL_UART_CLEAR_OREFLAG(huart); // Overrun
		__HAL_UART_CLEAR_FEFLAG(huart);  // Framing
		__HAL_UART_CLEAR_NEFLAG(huart);  // Noise
		HAL_UART_Receive_IT(&huart1, &rxChar, 1);
	}
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
	MX_LPUART1_UART_Init();

	HAL_UART_AbortReceive_IT(&hlpuart1);
	HAL_UART_AbortReceive_IT(&huart1);

	ep_show_intro();
	HAL_Delay(10000);
	intro_done = 1;
	HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);

	EP_BootSoft();

	log_init(&huart1);
	HAL_UART_Receive_IT(&huart1, &rxChar, 1);
	log_printf("Received From L432KC to WB55RG: %s\r\n", rxBuffer);



	HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);

	// Démarre la réception LPUART1 (depuis L432KC)
	rxlen = 0; line_ready = 0;
	//HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);

	last = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{


		if (messageReady)
		{
			log_printf("Received From L432KC to WB55RG: %s\r\n", rxBuffer);

			// 1) Copie dans rxbuf → format compatible parse_values()
			int n = snprintf(rxbuf, RXBUF_SZ, "#%s\n", rxBuffer);

			// 2) Signale à la logique WaveChart qu’une ligne est prête
			rxlen = n;
			line_ready = 1;

			// 3) Reset flags
			messageReady = 0;
			indexRx = 0;
			memset(rxBuffer, 0, sizeof(rxBuffer));
		}

		uint32_t v1, v2, v3, hid;
		if (line_ready) {

			// ----- 1) commande spéciale : SETVTH -----
			if (strcmp(rxbuf, "SETVTH") == 0) {
				ui_mode = MODE_ASK_VTH;
				ep_show_ask_vth();

				rxlen = 0;
				line_ready = 0;
				continue;
			}

			// ----- 2) si on attend une valeur pour Vth -----
			if (ui_mode == MODE_ASK_VTH) {
				uint32_t newVth_mV = strtoul(rxbuf, NULL, 10);

				if (newVth_mV > 100 && newVth_mV < 5000) {
					char cmd[32];
					snprintf(cmd, sizeof(cmd), "#SETVTH=%lu\n", newVth_mV);
					HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
				}
				ui_mode = MODE_DISPLAY;
				rxlen = 0;
				line_ready = 0;
				continue;
			}


			//uint32_t v1, v2, v3, hid;
			if (parse_values(rxbuf, &v1, &v2, &v3, &hid)) {
				pv1 = v1; pv2 = v2; pv3 = v3; pv4 = hid;
			}
			rxlen = 0; line_ready = 0;
			//HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

			uint32_t vgs = (v1 > v2 ? v1 - v2 : 0);
			uint32_t vds = (v3 > v2 ? v3 - v2 : 0);
			if (vgs < VTH) {
				HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(GPIOB, LD3_Pin);

			}
			else if (vds <= 0.02f) {
				HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);
			}
			else {
				float vgt = vgs - VTH;
				if (vds <= vgt){
					HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);
					HAL_GPIO_TogglePin(GPIOB, LD2_Pin);

				}
				else{
					HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin, GPIO_PIN_RESET);
					HAL_GPIO_TogglePin(GPIOB, LD1_Pin);

				}
			}
		}


		// Rafraîchit l’écran toutes les ~2 s (évite ghosting/lenteur e-paper)
		if (intro_done && HAL_GetTick() - last > 10000) {
			ep_show_values(pv1, pv2, pv3, pv4);
			last = HAL_GetTick();
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	// USART1 -> écran sur PB6/PB7

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;   // PB6/PB7 = AF7 pour USART1
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// LPUART1 -> lien L432KC sur PB10/PB11
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;  // PB10/PB11 = AF8 pour LPUART1
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
