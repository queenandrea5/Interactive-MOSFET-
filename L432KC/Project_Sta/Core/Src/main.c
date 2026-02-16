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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VREF     3.30f
#define VTH      1.80f    // seuil NMOS « pédagogique »
#define K_NMOS   0.5f     // facteur de transconductance (échelle visuelle)
#define ID_MAX   0.20f    // courant de normalisation pour la barre



typedef enum {
	MOS_OFF = 0,   // vGS < VTH  -> transistor bloqué
	MOS_TRIODE,    // vGS >= VTH et vDS <= vGS - VTH
	MOS_CHANNEL_ONLY,  // VGS ≥ VTH et VDS = 0
	MOS_SAT        // vGS >= VTH et vDS >  vGS - VTH
} MosRegime;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CHAIN 4        // 4 modules en chaîne
#define MAX_ROWS  8        // 8 lignes
#define CHANNEL_COLS  32
#define REG_NOOP        0x00
#define REG_DIGIT0      0x01
#define REG_DIGIT7      0x08
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY   0x0A
#define REG_SCAN_LIMIT  0x0B
#define REG_SHUTDOWN    0x0C
#define REG_DISPLAYTEST 0x0F
#define MAX_CS_GPIO_Port   GPIOB
#define MAX_CS_Pin         GPIO_PIN_6
#define MSG_SIZE 128
#define DELAY_MIN_MS  5u     // courant faible -> petit délai
#define DELAY_MAX_MS  120u   // courant fort  -> grand délai
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t light = 0;
static MosRegime mos_state = MOS_OFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MAX_SendAll(uint8_t reg[MAX_CHAIN], uint8_t data[MAX_CHAIN]);

static inline void MAX_CS_L(void);
static inline void MAX_CS_H(void);
static void MAX_Broadcast(uint8_t reg, uint8_t data);
static void MAX_Init(void);
static uint8_t fb[MAX_ROWS][MAX_CHAIN];

static void MAX_PushFB(void);
static void FB_SetColumn(uint8_t module, uint8_t col /*0..7*/, uint8_t height /*0..8*/);

static void FB_Clear(void);

uint8_t rxAck;
uint8_t ackBuffer[30]; // pour recevoir "ACK\r\n"
uint8_t rxBuffer[100];

uint8_t ackIndex = 0;
volatile uint8_t ackReady = 0;



void display_channel(uint8_t channel[8][32]);

void fillTriangleToRectangle_Animated(uint8_t channel[8][32], uint32_t delay_ms);

int isChannelFull(uint8_t channel[8][32]);

void setIntensity(uint8_t intensity);

static void channel_clear(uint8_t channel[8][32], uint8_t value);

void animateTriodeStep(uint8_t channel[8][32]);

uint32_t computeTriodePeriod(float id,  float id_max);

static uint32_t delay_from_id(float id,  float pct);

void fillTriangleWithPinch(uint8_t channel[8][32],float vgs,float vds,uint32_t delay_ms);

void displayPinnedChannel_Animated(uint8_t channel[8][32], float vgs, float vds, float vth, float vds_max, uint32_t delay_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void MAX_CS_L(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); }
static inline void MAX_CS_H(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   }

static void MAX_SendAll(uint8_t reg[MAX_CHAIN], uint8_t data[MAX_CHAIN]){
	uint8_t frame[2*MAX_CHAIN];
	// envoie m = MAX_CHAIN-1 → 0
	for (int m = MAX_CHAIN-1, i = 0; m >= 0; --m, ++i){
		frame[2*i+0] = reg[m];
		frame[2*i+1] = data[m];
	}
	MAX_CS_L();
	HAL_SPI_Transmit(&hspi1, frame, sizeof(frame), 100);
	MAX_CS_H();
}
static void MAX_Broadcast(uint8_t reg, uint8_t data)
{
	uint8_t R[MAX_CHAIN], D[MAX_CHAIN];
	for (int i=0; i<MAX_CHAIN; ++i){ R[i]=reg; D[i]=data; }
	MAX_SendAll(R,D);
}
static void MAX_Init(void)
{
	MAX_CS_H();
	HAL_Delay(10);
	MAX_Broadcast(REG_DISPLAYTEST, 0x00); // test OFF
	MAX_Broadcast(REG_SCAN_LIMIT,  0x07); // 8 digits (0..7)
	MAX_Broadcast(REG_DECODE_MODE, 0x00); // no decode (matrice)
	MAX_Broadcast(REG_INTENSITY,   0x04); // luminosité (0..0x0F)
	MAX_Broadcast(REG_SHUTDOWN,    0x01); // normal operation

	//log_init(&huart1);
	//log_printf("Initiation L432KC MUST SEND DATA TO WB55RG\r\n");
	// Clear: écrire 0x00 sur les 8 DIGITx
	for (uint8_t row=REG_DIGIT0; row<=REG_DIGIT7; ++row){
		uint8_t R[MAX_CHAIN], D[MAX_CHAIN];
		for (int m=0;m<MAX_CHAIN;++m){ R[m]=row; D[m]=0x00; }
		MAX_SendAll(R,D);
	}
}

/* Frame buffer 8 lignes x 4 matrices (chaque data = 8 bits, 1=LED allumée).
   fb[row][module] */


static uint8_t fb[8][MAX_CHAIN];


uint8_t channel[8][32];

void setIntensity(uint8_t intensity){
	if (intensity > 15) intensity = 15;

	// Envoi de la luminosité à tous les modules MAX7219
	MAX_Broadcast(REG_INTENSITY, intensity);
}




void fillTriangleToRectangle_Animated(uint8_t channel[8][32], uint32_t delay_ms){
	light = 0;
	for (int col = 0; col < 32; col++)
	{
		int height = (col * 8) / 31;  // hauteur du triangle (0 -> 8)

		// Remplissage depuis la BASE (row = 7) vers le HAUT (row = 0)
		for (int row = 0; row < 8; row++)
		{
			int inverted_row = 7 - row;   // 7 = bas, 0 = haut

			if (row < height){
				channel[inverted_row][col] = 0; // off

			}
			else{
				channel[inverted_row][col] = 1; //on
				light++;
			}

		}

		uint8_t intensity = (light * 15) / 192;
		setIntensity(intensity);

		display_channel(channel);
		HAL_Delay(delay_ms);
	}
}



static uint32_t delay_from_id(float id, float pct){


	// plus id est grand -> delay PLUS PETIT
	float d = (float)DELAY_MAX_MS - pct * (float)(DELAY_MAX_MS - DELAY_MIN_MS);

	return (uint32_t)(d + 0.5f);
}



void animateTriodeStep(uint8_t channel[8][32]){
	static uint8_t col = 0;


	int height = (col * 8) /32;

	for (int row = 0; row < 8; row++) {
		int inv = 7 - row;
		channel[inv][col] = (row < height) ? 0 : 1;
	}

	display_channel(channel);

	col++;
	if (col >= 32) {
		col = 0;   // on recommence → animation cyclique
	}
}



uint32_t computeTriodePeriod(float id, float id_max){
	// sécurité

	// ID faible → lent, ID fort → rapide
	uint32_t Tmin = 30;   // ms
	uint32_t Tmax = 500;  // ms

	float ratio = id / id_max;   // 0 → 1
	return (uint32_t)(Tmax - ratio * (Tmax - Tmin));
}



int isChannelFull(uint8_t channel[8][32]){
	for (int row = 0; row < 8; row++)
		for (int col = 0; col < 32; col++)
			if (channel[row][col] == 0)
				return 0;

	return 1; // toutes les LEDs sont à 1
}


static void FB_Clear(void)
{
	memset(fb, 0, sizeof(fb));
}

static void FB_SetColumn(uint8_t module, uint8_t col, uint8_t height){
	if(module>=MAX_CHAIN || col>7) return;
	if(height>8) height=8;
	for(uint8_t r=0;r<8;r++){
		if(r<height) fb[r][module] |=  (1u<<col);
		else         fb[r][module] &= ~(1u<<col);
	}
}

static void MAX_PushFB(void){
	for(uint8_t row=0;row<8;row++){
		uint8_t R[MAX_CHAIN], D[MAX_CHAIN];
		for(int m=0;m<MAX_CHAIN;m++){
			R[m] = 0x01 + row;    // DIGIT0..DIGIT7 = 0x01..0x08
			D[m] = fb[row][m];
		}
		MAX_SendAll(R,D);
	}
}



/* Lis successivement A0(ADC1_IN5), A1(ADC1_IN6), A2(ADC1_IN8) */


/* Mappe 0..4095 vers 0..8 */

/*static inline uint8_t map_0_4095_to_0_8(uint16_t v){
	return (uint8_t)((uint32_t)v * 9u / 4096u); // 0..8
}*/

static float last_vs_eff = 0.0f;

__attribute__((aligned(4))) volatile uint16_t adc_buf[3] = {0,0,0};

uint8_t wiping[4] = {0,0,0,0};    // 1 = en phase d’effacement pour ce module
uint8_t wipe_row[4] = {0,0,0,0};
volatile uint16_t g_a0, g_a1, g_a2;


static uint16_t adc_read_one(uint32_t channel)
{
	ADC_ChannelConfTypeDef s = {0};
	s.Channel = channel;
	s.Rank = ADC_REGULAR_RANK_1;
	s.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;   // un peu long = stable
	s.SingleDiff = ADC_SINGLE_ENDED;
	s.OffsetNumber = ADC_OFFSET_NONE;
	s.Offset = 0;

	HAL_ADC_ConfigChannel(&hadc1, &s);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);        // 10 ms max
	uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return v;
}

static void adc_read_3(uint16_t *a0, uint16_t *a1, uint16_t *a2)
{
	// Nucleo L432KC : PA0=ADC_IN5, PA1=ADC_IN6, PA3=ADC_IN8  (vérifié)
	*a0 = adc_read_one(ADC_CHANNEL_5);  // PA0
	*a1 = adc_read_one(ADC_CHANNEL_6);  // PA1
	*a2 = adc_read_one(ADC_CHANNEL_8);  // PA3
}


static void channel_clear(uint8_t channel[8][32], uint8_t value)
{
	for (int r = 0; r < 8; r++)
		for (int c = 0; c < 32; c++)
			channel[r][c] = value;
}

void display_channel(uint8_t channel[8][32]){
	FB_Clear();

	for (int col = 0; col < 32; col++)
	{
		int module = col / 8;      // 0 à 3
		int col_in_module = col % 8;

		for (int row = 0; row < 8; row++)
		{
			if (channel[row][col])
				fb[row][module] |= (1 << col_in_module);
			else
				fb[row][module] &= ~(1 << col_in_module);
		}
	}

	MAX_PushFB();
}



void displayPinnedChannel_Animated(uint8_t channel[8][32],
		float vgs, float vds,
		float vth, float vds_max,
		uint32_t delay_ms)
{
	// 1) écran vide
		for (int r = 0; r < 8; r++)
			for (int c = 0; c < 32; c++)
				channel[r][c] = 0;

		// facteur de pincement global
		float pinch = (vds - (vgs - vth)) / vds_max;
		if (pinch < 0.0f) pinch = 0.0f;
		if (pinch > 1.0f) pinch = 1.0f;

		// 2) affichage progressif source → drain
		for (int col = 0; col < 32; col++)
		{
			// hauteur triode de référence
			int base_h = (col * 8) / 31;

			// position normalisée
			float x = (float)col / 31.0f;

			// pincement → on AUGMENTE le seuil
			int delta = (int)(pinch * x * 8.0f + 0.5f);

			int h = base_h + delta;
			if (h > 8) h = 8;

			// allumer UNIQUEMENT le canal
			for (int row = h; row < 8; row++)
			{
				int inv = 7 - row;
				channel[inv][col] = 1;
			}

			display_channel(channel);
			HAL_Delay(delay_ms);
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

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	MAX_Init();

	log_init(&huart2);
	log_printf("### LOG_PRINTF TEST ###\r\n");
	//log_printf("Initiation L432KC MUST SEND DATA TO WB55RG\r\n");
	// sanity tests

	MAX_Broadcast(REG_DISPLAYTEST, 0x00);
	FB_Clear();
	MAX_PushFB();   // tout éteint

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		uint16_t a0, a1, a2;
		adc_read_3(&a0, &a1, &a2);   // <-- vrai polling, 3 lectures

		g_a0 = a0; g_a1 = a1; g_a2 = a2;   // pour debugger en live

		float vg = (a0 * VREF) / 4095.0f;
		float vs = (a1 * VREF) / 4095.0f;
		float vd = (a2 * VREF) / 4095.0f;

		// Physique simple : VS ne peut pas dépasser VD
		float vs_eff = vs;
		if (vs_eff > vd - 0.05f) vs_eff = vd - 0.05f;               // clamp
		if (vs_eff < 0.0f)       vs_eff = 0.0f;

		// MOSFET
		float vgs = vg - vs_eff;
		float vds = vd - vs_eff;
		float id  = 0.0f;

		float vgt = vgs - VTH;
		float id_max = 0.5f * K_NMOS * vgt * vgt;
		if (id_max < 1e-6f) id_max = 1e-6f;

		if (vgs > VTH && vds > 0.0f) {

			if (vds < vgt) { // linéaire
				id = K_NMOS * (vgt * vds - 0.5f * vds * vds);
			} else {         // saturation
				id = 0.5f * K_NMOS * vgt * vgt;
			}
		}else{
			id = 0.0f;
		}

		// “VS ne peut plus augmenter quand tout est allumé”:
		// si la barre courant est pleine, on gèle VS (comportement pédagogique)

		//if (id < 0.0f) id = 0.0f;
		//if (id > ID_MAX) id = ID_MAX;

		// --- normalisation bargraph (0..1) ---
		float pct = id / id_max;
		if (pct > 1.0f) pct = 1.0f;   // sécurité


		//log_printf("ID_MAX%.2f ; ID%.2f\r\n",id_max, id);

		// “VS ne peut plus augmenter quand tout est allumé” (pédagogique)
		if (pct >= 1.0f && vs > last_vs_eff) {
			vs_eff = last_vs_eff;
		} else {
			last_vs_eff = vs_eff;
		}

		/*float pct = id / ID_MAX; if (pct > 1.0f) pct = 1.0f; if (pct < 0.0f) pct = 0.0f;
		if (pct >= 1.0f && vs > last_vs_eff) {
			vs_eff = last_vs_eff; // empêche VS de monter davantage à l’affichage
		} else {
			last_vs_eff = vs_eff;
		}*/






		MosRegime new_state;
		int o = -1;

		if (vgs < VTH) {
			new_state = MOS_OFF;
			o=0;
		}
		else if (vds <= 0.02f) {
			new_state = MOS_CHANNEL_ONLY;
		}
		else {
			float vgt = vgs - VTH;
			if (vds <= vgt){
				new_state = MOS_TRIODE;
				o=1;
			}

			else{
				new_state = MOS_SAT;
				o=3;
			}

		}



		uint32_t delay_ms = delay_from_id(id, pct);

		// Si le régime change, on met à jour le dessin du canal
		if (new_state != mos_state) {
			mos_state = new_state;



			switch (mos_state) {

			case MOS_OFF:
				channel_clear(channel, 0); // vGS < VTH -> aucune LED sur les 3 premiers blocs
				display_channel(channel);
				setIntensity(0);            // intensité mini
				break;

			case MOS_CHANNEL_ONLY:

				fillTriangleToRectangle_Animated(channel, 0);
				break;

			case MOS_TRIODE:
				// clear écran UNE fois
				for (int r = 0; r < 8; r++)
					for (int c = 0; c < 32; c++)
						channel[r][c] = 0;
				HAL_Delay(500);

				fillTriangleToRectangle_Animated(channel, delay_ms);

				//setIntensity(8);    // par ex. intensité moyenne


				break;


			case MOS_SAT:
				o = 3;

				HAL_Delay(200);
				displayPinnedChannel_Animated(channel,
						vgs, vds,
						VTH, 3.3f,
						delay_ms);

				break;
			}

			log_printf("%d\r\n", o);
		}



		if (mos_state == MOS_TRIODE){
			for (int r = 0; r < 8; r++)
				for (int c = 0; c < 32; c++)
					channel[r][c] = 0;   // on repart d'un écran vide
			HAL_Delay(500);
			//uint32_t delay_ms = delay_from_id(id, pct);
			// triangle qui progresse de la source vers le drain
			fillTriangleToRectangle_Animated(channel, delay_ms);
		}
		if(mos_state == MOS_SAT){

			HAL_Delay(200);
			displayPinnedChannel_Animated(channel,
					vgs, vds,
					VTH, 3.3f,
					delay_ms);

		}

		char msg[MSG_SIZE];

		// conversion Volts -> mV
		uint32_t vg_mV = (uint32_t)(vg * 1000.0f);
		uint32_t vs_mV = (uint32_t)(vs_eff * 1000.0f);
		uint32_t vd_mV = (uint32_t)(vd * 1000.0f);
		uint32_t Id_mA = (uint32_t)(id * 1000.0f + 0.5f);
		//uint32_t Id_mA = (uint32_t)(pct*1000.0f+ 0.5f);
		// Construction de la trame EXACTE compatible parse_values()
		snprintf(msg, sizeof(msg),
				"#VG=%lu VS=%lu VD=%lu Id=%lu\r\n",
				(unsigned long)vg_mV,
				(unsigned long)vs_mV,
				(unsigned long)vd_mV,
				(unsigned long)Id_mA);

		// Envoi UART
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		// Envoi UART VERS WB55
		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		//log_printf("Message Sent : %s\r\n", msg+1);
		HAL_Delay(500);
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
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
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
