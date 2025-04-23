/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdbool.h"
#include "crc32.h"
#include "SSD1500.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "KDU.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LCD_WIDTH 122
#define LCD_HEIGHT 4
uint8_t image[LCD_HEIGHT][LCD_WIDTH];

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
#define MIN(i, j) (((i) < (j)) ? (i) : (j))
#define MAX(i, j) (((i) > (j)) ? (i) : (j))

#define KeypadDebounce 5

extern const unsigned char epd_bitmap_Bootsplash_copy[488];
extern const unsigned char font5x7[1330];

#include "GLCD.h"

DisplayData_t displaydata[2];
volatile DisplayData_t *validdisplay;
volatile DisplayData_t *receivingdisplay;

const char Default_TopLine[32] = "H AF ---- V60 48WBHA T DP- DMR";
const char Default_BotLine[32] = "  BT ---- SCN 123456 T SPX  AM";
const char Default_Freq1Line[21] = " LD1NR Loc. 06 ------";
const char Default_Freq2Line[21] = " ENKJ Traff 01 ------";

char UART1TXBuffer[2048];
char UART1_RX_Buffer[1024];
char UART1_RX_Buffer_Decode[512];
volatile bool ReceivedDisplayData;

reset_cause_t rstcause;

typedef struct
{
	uint8_t SerialNo;
	uint8_t pad;
	uint16_t LCD_Contrast;
	uint32_t _CRC;
} FRAM_Data_t;

FRAM_Data_t poweronfram;
bool poweronframvalid;
FRAM_Data_t writefram;
bool writefram_;
bool writeframok;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes =
{ .name = "defaultTask", .stack_size = 1024 * 4, .priority =
		(osPriority_t) osPriorityNormal, };
/* Definitions for serialTask */
osThreadId_t serialTaskHandle;
const osThreadAttr_t serialTask_attributes =
{ .name = "serialTask", .stack_size = 512 * 4, .priority =
		(osPriority_t) osPriorityLow, };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartserialTask(void *argument);

/* USER CODE BEGIN PFP */

void InitDisplayData(DisplayData_t *data)
{
	memcpy(&data->TopLine, &Default_TopLine, sizeof(Default_TopLine));
	memcpy(&data->BottomLine, &Default_BotLine, sizeof(Default_BotLine));
	memcpy(&data->Frequency1Line, &Default_Freq1Line,
			sizeof(Default_Freq1Line));
	memcpy(&data->Frequency2Line, &Default_Freq2Line,
			sizeof(Default_Freq2Line));
	data->Arrow1 = 1;
	data->Arrow2 = 0;
	data->TextMode = 0;
	data->Line1BarGraph = 15;
	data->Line1BarGraph2 = -1;
	data->Line2BarGraph = 20;
	data->Line2BarGraph = -1;
	data->TopBarGraph = 4;
	data->TopBarGraph = -1;
	data->BottomBarGraph = 18;
	data->BottomBarGraph = -1;
	data->LCD_Contrast = 2700;
	data->LED = 2;
	data->Backlight = 4095;
	data->Backlight_Minimum = 100;
}

int SerializeArray(uint8_t source, uint8_t datatype, uint16_t size,
		const char *sourceptr, char *outptr, size_t outsize, bool newline)
{
	int count = 0;

	uint32_t crc = xcrc32((const unsigned char*) sourceptr, size, 0);
	// print the start byte and header info
	// : [source] [data type] [size of payload - header] [crc-32] |
	count = snprintf(outptr, outsize, ":%02X%02X%04X%08lX|", source, datatype,
			size, crc);

	int loopcount = 0;

	for (int i = 0; i < size; i++)
	{
		// print one byte as ASCII hex
		// starting at the end of the size field
		loopcount += snprintf((outptr + count) + loopcount,
				outsize - (i * 2) - count, "%02X", (uint8_t) *sourceptr);
		// go to next byte
		sourceptr++;
	}
	// update our array position
	count += loopcount;
	// add end-of-line
	if (newline)
		count += snprintf(outptr + count, outsize - count, "\r\n");
	else
		count += snprintf(outptr + count, outsize - count, "\r");

	return count;
}

void USART1_Process_New_Data(const char *data, size_t len)
{
	static int RXState = 0;
	static int RXBufferIdx = 0;
	const char framestart = ':';
	const char framestop = '\r';
	const int sourceoffset = 1;
	const int messageoffset = 3;
	const int sizeoffset = 5;
	const int crcoffset = 9;
	const int pipeoffset = 17;
	const int payloadoffset = 18;
	for (int i = 0; i < len; i++)
	{
		char newchar = data[i];
		if (newchar == framestart)
		{
			RXState = 1;
			memset(&UART1_RX_Buffer_Decode, 0, sizeof(UART1_RX_Buffer_Decode));
			RXBufferIdx = 0;
		}

		// accumulating characters
		if (RXState == 1)
		{
			// abort if we receive invalid characters
			// allowed characters are: decimal 32-126 (printable range), newline, and carriage return
			if (newchar < 32 || newchar > 126)
				if (newchar != '\n' && newchar != '\r')
					RXState = 0;
			UART1_RX_Buffer_Decode[RXBufferIdx] = newchar;
			RXBufferIdx++;
			if (RXBufferIdx >= sizeof(UART1_RX_Buffer_Decode))
				RXBufferIdx = sizeof(UART1_RX_Buffer_Decode) - 1;
		}

		if (newchar == framestop && UART1_RX_Buffer_Decode[pipeoffset] == '|'
				&& RXState == 1)
		{
			RXState = 2;
			char buf[9];
			memset(buf, 0, sizeof(buf));
			char *end;
			memcpy(buf, &UART1_RX_Buffer_Decode[sourceoffset], 2);
			// decode source, 0=Radio
			int source = strtoul(buf, &end, 16);

			memcpy(buf, &UART1_RX_Buffer_Decode[messageoffset], 2);
			// message ID
			int message = strtoul(buf, &end, 16);

			memcpy(buf, &UART1_RX_Buffer_Decode[sizeoffset], 4);
			// check size of data we've got
			int payloadsize = strtoul(buf, &end, 16);

			memcpy(buf, &UART1_RX_Buffer_Decode[crcoffset], 8);

			uint32_t rxcrc = strtoul(buf, &end, 16);

			memset(buf, 0, sizeof(buf));

			// handle the DisplayData_t message
			if (source == 3 && message == 1
					&& payloadsize == sizeof(DisplayData_t))
			{
				for (int i = 0; i < sizeof(DisplayData_t); i++)
				{
					memcpy(buf,
							&UART1_RX_Buffer_Decode[(i * 2) + payloadoffset],
							2);
					uint8_t byteval = strtoul(buf, &end, 16);
					*((uint8_t*) receivingdisplay + i) = byteval;
				}
				uint32_t crc = xcrc32((uint8_t*) receivingdisplay,
						sizeof(DisplayData_t), 0);
				if (crc == rxcrc)
				{
					// on receiving a valid command, we swap the valid/receiving pointers
					// this avoids any copies etc.
					// though the display could glitch for one cycle if it's swapped during rendering
					// this seems unproblematic, since it would be corrected on the next frame
					volatile DisplayData_t *oldvalid = validdisplay;
					validdisplay = receivingdisplay;
					receivingdisplay = oldvalid;
					ReceivedDisplayData = true;
				}

			}
			/*else if (source == 3 && message == 2 && payloadsize == sizeof(RF5980_CMD2_t))
			 {
			 for (int i = 0; i < sizeof(RF5980_CMD2_t); i++)
			 {
			 memcpy(buf, &UART1_RX_Buffer_Decode[(i*2)+payloadoffset], 2);
			 uint8_t byteval = strtoul(buf, &end, 16);
			 *((uint8_t*)&LastCMD2 + i) = byteval;
			 }
			 uint32_t crc = xcrc32((uint8_t*)&LastCMD2, sizeof(RF5980_CMD2_t), 0);
			 LastCMD2CRCOk = crc == rxcrc;
			 LastCMD2Processed = !LastCMD2CRCOk;
			 }*/
		}
	}
}

void USART1_DMA_New_Data(bool reset)
{
	static size_t old_pos;
	size_t pos;

	if (reset)
	{
		old_pos = 0;
		return;
	}

	/* Calculate current position in buffer */
	pos = ARRAY_LEN(UART1_RX_Buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
	if (pos != old_pos)
	{ /* Check change in received data */
		if (pos > old_pos)
		{ /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			USART1_Process_New_Data(&UART1_RX_Buffer[old_pos], pos - old_pos);
		}
		else
		{
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			USART1_Process_New_Data(&UART1_RX_Buffer[old_pos],
					ARRAY_LEN(UART1_RX_Buffer) - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0)
			{
				USART1_Process_New_Data(&UART1_RX_Buffer[0], pos);
			}
		}
	}
	old_pos = pos; /* Save current position as old */

	/* Check and manually update if we reached end of buffer */
	if (old_pos == ARRAY_LEN(UART1_RX_Buffer))
	{
		old_pos = 0;
	}
}

bool LoadFRAM(int offset, int bank, FRAM_Data_t *data)
{
	// load F-RAM
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0xA0 + bank, 0,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) data, sizeof(FRAM_Data_t), 10);
	uint32_t calculatedcrc = xcrc32((const unsigned char*) data,
			sizeof(FRAM_Data_t) - sizeof(uint32_t), 0);
	return (calculatedcrc == data->_CRC) && status == HAL_OK;
}

bool WriteFRAM(int offset, int bank, FRAM_Data_t *data)
{
	// calculate CRC for write
	data->_CRC = xcrc32((const unsigned char*) data,
			sizeof(FRAM_Data_t) - sizeof(uint32_t), 0);
	// write data
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, 0xA0 + bank, offset,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) data, sizeof(FRAM_Data_t), 10);
	// read back
	FRAM_Data_t verify;
	HAL_I2C_Mem_Read(&hi2c1, 0xA0 + bank, 0, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &verify, sizeof(FRAM_Data_t), 10);
	// calculate CRC
	uint32_t readbackcrc = xcrc32((const unsigned char*) &verify,
			sizeof(FRAM_Data_t) - sizeof(uint32_t), 0);
	// return true if both readback ok and write ok
	return (readbackcrc == data->_CRC) && status == HAL_OK;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TimerDelay_Init(void)
{
	uint32_t gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	htim2.Init.Prescaler = gu32_ticks - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_Base_Start(&htim2);
}

void delay_us(uint16_t au16_us)
{
	osKernelLock();
	htim2.Instance->CNT = 0;
	while (htim2.Instance->CNT < au16_us)
		;
	osKernelUnlock();
}

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause
reset_cause_t reset_cause_get(void)
{
	reset_cause_t reset_cause;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
	{
		reset_cause = RESET_CAUSE_LOW_POWER_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
	{
		reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{
		reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
	{
		// This reset is induced by calling the ARM CMSIS
		// `NVIC_SystemReset()` function!
		reset_cause = RESET_CAUSE_SOFTWARE_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
	{
		reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
	{
		reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
	}
	// Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
	// ensure first that the reset cause is NOT a POR/PDR reset. See note
	// below.
	/*else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
	 {
	 reset_cause = RESET_CAUSE_BROWNOUT_RESET;
	 }*/
	else
	{
		reset_cause = RESET_CAUSE_UNKNOWN;
	}

	// Clear all the reset flags or else they will remain set during future
	// resets until system power is fully removed.
	__HAL_RCC_CLEAR_RESET_FLAGS();

	return reset_cause;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	__HAL_DBGMCU_FREEZE_IWDG();
	__HAL_DBGMCU_FREEZE_WWDG();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	rstcause = reset_cause_get();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_DAC_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_CRC_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	validdisplay = &displaydata[0];
	receivingdisplay = &displaydata[1];

	// try to read the F-RAM
	poweronframvalid = LoadFRAM(0, 0, &poweronfram);
	if (!poweronframvalid)
		poweronframvalid = LoadFRAM(0, 1, &poweronfram);

	if (!poweronframvalid)
		memset(&poweronfram, 0, sizeof(FRAM_Data_t));
	else
		memcpy(&writefram, &poweronfram, sizeof(FRAM_Data_t));

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
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of serialTask */
	serialTaskHandle = osThreadNew(StartserialTask, NULL,
			&serialTask_attributes);

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */

	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

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
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

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

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigOC.Pulse = 4500;
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

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4095;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	huart1.Init.BaudRate = 230400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LCD_WR_L_Pin | LCD_OE_L_Pin | LCD_RD_L_Pin | LCD_CS1_L_Pin
					| LCD_CS2_L_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LCD_DATA_COMMAND_Pin | LCD_RESET_L_Pin | LCD_DIR_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, HEAT1_Pin | HEAT2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LCD_WR_L_Pin LCD_DATA_COMMAND_Pin LCD_RESET_L_Pin LCD_OE_L_Pin
	 LCD_DIR_Pin LCD_RD_L_Pin LCD_CS1_L_Pin LCD_CS2_L_Pin */
	GPIO_InitStruct.Pin = LCD_WR_L_Pin | LCD_DATA_COMMAND_Pin | LCD_RESET_L_Pin
			| LCD_OE_L_Pin | LCD_DIR_Pin | LCD_RD_L_Pin | LCD_CS1_L_Pin
			| LCD_CS2_L_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DB0_Pin DB1_Pin DB2_Pin DB3_Pin
	 DB4_Pin DB5_Pin DB6_Pin DB7_Pin */
	GPIO_InitStruct.Pin = DB0_Pin | DB1_Pin | DB2_Pin | DB3_Pin | DB4_Pin
			| DB5_Pin | DB6_Pin | DB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : KP_COL1_Pin KP_COL2_Pin KP_COL3_Pin KP_R3_Pin
	 KP_R4_Pin KP_R5_Pin KP_R6_Pin KP_RFU_Pin
	 KP_R1_Pin KP_R2_Pin */
	GPIO_InitStruct.Pin = KP_COL1_Pin | KP_COL2_Pin | KP_COL3_Pin | KP_R3_Pin
			| KP_R4_Pin | KP_R5_Pin | KP_R6_Pin | KP_RFU_Pin | KP_R1_Pin
			| KP_R2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : HEAT1_Pin HEAT2_Pin */
	GPIO_InitStruct.Pin = HEAT1_Pin | HEAT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

int16_t PWM;
int16_t bias;

uint8_t lcdret;

KDU_KeyPress_t CurrentKey;

void ReadRows(uint8_t *col)
{
	// this delay is probably not needed
	osDelay(2);
	*col = 0;
	*col |= HAL_GPIO_ReadPin(KP_R1_GPIO_Port, KP_R1_Pin);
	*col |= HAL_GPIO_ReadPin(KP_R2_GPIO_Port, KP_R2_Pin) << 1;
	*col |= HAL_GPIO_ReadPin(KP_R3_GPIO_Port, KP_R3_Pin) << 2;
	*col |= HAL_GPIO_ReadPin(KP_R4_GPIO_Port, KP_R4_Pin) << 3;
	*col |= HAL_GPIO_ReadPin(KP_R5_GPIO_Port, KP_R5_Pin) << 4;
	*col |= HAL_GPIO_ReadPin(KP_R6_GPIO_Port, KP_R6_Pin) << 5;
}

void ScanKeypad(KDU_KeyPress_t *keys)
{
	/* Slow but pretty good keypad scanner
	 * we use tri-stating and pulldowns to properly scan it
	 * As such, it needs to be a little slow to work.
	 * It seems good up to 3 arbitrary keys, but is ambiguous above that*/
	uint8_t col[3];
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = KP_R3_Pin | KP_R4_Pin | KP_R5_Pin | KP_R6_Pin
			| KP_RFU_Pin | KP_R1_Pin | KP_R2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KP_COL1_Pin | KP_COL2_Pin | KP_COL3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KP_COL1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(KP_COL1_GPIO_Port, KP_COL1_Pin, ENABLE);
	ReadRows(&col[0]);
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KP_COL2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(KP_COL2_GPIO_Port, KP_COL2_Pin, ENABLE);
	ReadRows(&col[1]);
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KP_COL3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(KP_COL3_GPIO_Port, KP_COL3_Pin, ENABLE);
	ReadRows(&col[2]);
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	keys->bits.Zero = (col[0] & 1) > 0;
	keys->bits.One = (col[0] & 2) > 0;
	keys->bits.Two = (col[0] & 4) > 0;
	keys->bits.Three = (col[0] & 8) > 0;
	keys->bits.LeftArrow = (col[0] & 16) > 0;
	keys->bits.RightArrow = (col[0] & 32) > 0;

	keys->bits.Vol_Plus = (col[1] & 1) > 0;
	keys->bits.Four = (col[1] & 2) > 0;
	keys->bits.Five = (col[1] & 4) > 0;
	keys->bits.Six = (col[1] & 8) > 0;
	keys->bits.CLR = (col[1] & 16) > 0;
	keys->bits.Pre_Plus = (col[1] & 32) > 0;

	keys->bits.Vol_Minus = (col[2] & 1) > 0;
	keys->bits.Seven = (col[2] & 2) > 0;
	keys->bits.Eight = (col[2] & 4) > 0;
	keys->bits.Nine = (col[2] & 8) > 0;
	keys->bits.ENT = (col[2] & 16) > 0;
	keys->bits.Pre_Minus = (col[2] & 32) > 0;
}

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // LCD 2 kHz Clock
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // LCD Backlight PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Keypad PWM
	PWM = 4095;
	bias = poweronfram.LCD_Contrast;

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

	TimerDelay_Init();

	HAL_GPIO_WritePin(LCD_RESET_L_GPIO_Port, LCD_RESET_L_Pin, ENABLE);
	HAL_GPIO_WritePin(LCD_RESET_L_GPIO_Port, LCD_RESET_L_Pin, DISABLE);
	HAL_GPIO_WritePin(LCD_RESET_L_GPIO_Port, LCD_RESET_L_Pin, ENABLE);

	InitLCD(0);
	InitLCD(1);

	HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, DISABLE);
	HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, DISABLE);

	uint8_t cnt = 0;

	FillLCD(0xff);

	InitDisplayData(&displaydata[0]);
	InitDisplayData(&displaydata[1]);

	// preset the contrast to auto to use FRAM settings
	validdisplay->LCD_Contrast = -1;

	volatile DisplayData_t *olddata = validdisplay;
	int bootsplashtimer = 300;
	int commtimer = 0;

	/* Infinite loop */
	for (;;)
	{
		HAL_IWDG_Refresh(&hiwdg);
		cnt++;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, bias);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		htim3.Instance->CCR1 = PWM;
		htim3.Instance->CCR2 = PWM / 8;

		PWM = validdisplay->Backlight;
		PWM = PWM > 4095 ? 4095 : PWM < 0 ? 0 : PWM;
		if (validdisplay->LCD_Contrast > 0)
			bias = validdisplay->LCD_Contrast;
		else
			bias = writefram.LCD_Contrast;
		bias = bias > 4095 ? 4095 : bias < 0 ? 0 : bias;

		// wait for new data, trying to sync processing with incoming data to reduce latency
		for (int i = 0; i < 5; i++)
		{
			if (validdisplay == olddata)
				osDelay(1);
			else
				break;
		}

		// check no-comm timer, used to go to bootsplash automatically
		commtimer++;
		if (validdisplay != olddata)
			commtimer = 0;

		olddata = validdisplay;

		// render image based on modes
		memset(&image, 0, sizeof(image));

		if (validdisplay->TextMode == 0 || validdisplay->TextMode == 1)
		{
			for (int i = 0; i < sizeof(validdisplay->Frequency1Line); i++)
				DisplayWriteCharacter5x7(&image[1][(i * 6)],
						&image[1][LCD_WIDTH - 1], (char*) &font5x7,
						validdisplay->Frequency1Line[i], false);
			for (int i = 0; i < sizeof(validdisplay->Frequency2Line); i++)
				DisplayWriteCharacter5x7(&image[2][(i * 6)],
						&image[2][LCD_WIDTH - 1], (char*) &font5x7,
						validdisplay->Frequency2Line[i], false);
		}
		if (validdisplay->TextMode == 1)
		{
			for (int i = 0; i < sizeof(validdisplay->TopLine); i++)
				DisplayWriteCharacter5x7(&image[0][(i * 6)],
						&image[0][LCD_WIDTH - 1], (char*) &font5x7,
						validdisplay->TopLine[i], false);
			for (int i = 0; i < sizeof(validdisplay->BottomLine); i++)
				DisplayWriteCharacter5x7(&image[3][(i * 6)],
						&image[3][LCD_WIDTH - 1], (char*) &font5x7,
						validdisplay->BottomLine[i], false);
		}

		if (validdisplay->TextMode == 0 || validdisplay->TextMode == 2
				|| validdisplay->TextMode == 3)
		{
			for (int i = 0; i < sizeof(validdisplay->TopLine); i++)
				DisplayWriteCharacter4x6(&image[0][1 + (i * 4)],
						&image[0][LCD_WIDTH - 1], validdisplay->TopLine[i]);
			for (int i = 0; i < sizeof(validdisplay->BottomLine); i++)
				DisplayWriteCharacter4x6(&image[3][1 + (i * 4)],
						&image[3][LCD_WIDTH - 1], validdisplay->BottomLine[i]);
		}
		if (validdisplay->TextMode == 2 || validdisplay->TextMode == 3)
		{
			for (int i = 0; i < sizeof(validdisplay->TopLine); i++)
				DisplayWriteCharacter4x6(&image[1][1 + (i * 4)],
						&image[1][LCD_WIDTH - 1],
						validdisplay->Frequency1Line[i]);
			for (int i = 0; i < sizeof(validdisplay->BottomLine); i++)
				DisplayWriteCharacter4x6(&image[2][1 + (i * 4)],
						&image[2][LCD_WIDTH - 1],
						validdisplay->Frequency2Line[i]);
		}

		if (validdisplay->TextMode == 0 || validdisplay->TextMode == 3)
		{
			if (validdisplay->Arrow1 != 0)
				DisplayWriteCharacter5x7(&image[1][0], &image[1][LCD_WIDTH - 1],
						(char*) &font5x7, 16,
						validdisplay->Arrow1 < 0 ? true : false);
			if (validdisplay->Arrow2 != 0)
				DisplayWriteCharacter5x7(&image[2][0], &image[2][LCD_WIDTH - 1],
						(char*) &font5x7, 16,
						validdisplay->Arrow2 < 0 ? true : false);
			// 13 wide bar graph
			if (validdisplay->TopBarGraph >= 0)
				DisplayDrawBarGraph(&image[0][22], &image[0][LCD_WIDTH - 1], 20,
						validdisplay->TopBarGraph, validdisplay->TopBarGraph2);
			if (validdisplay->BottomBarGraph >= 0)
				DisplayDrawBarGraph(&image[3][22], &image[3][LCD_WIDTH - 1], 20,
						validdisplay->BottomBarGraph,
						validdisplay->BottomBarGraph2);

			if (validdisplay->Line1BarGraph >= 0)
				DisplayDrawBarGraph(&image[1][89], &image[1][LCD_WIDTH - 1], 31,
						validdisplay->Line1BarGraph,
						validdisplay->Line1BarGraph2);
			if (validdisplay->Line2BarGraph >= 0)
				DisplayDrawBarGraph(&image[2][89], &image[2][LCD_WIDTH - 1], 31,
						validdisplay->Line2BarGraph,
						validdisplay->Line2BarGraph2);
		}

		// show bootsplash on startup and when loss of comms

		if (bootsplashtimer > 0 || commtimer > 200)
		{
			bootsplashtimer--;

			memcpy(&image[0][0], &epd_bitmap_Bootsplash_copy[0], LCD_WIDTH);
			memcpy(&image[1][0], &epd_bitmap_Bootsplash_copy[LCD_WIDTH],
					LCD_WIDTH);
			memcpy(&image[2][0], &epd_bitmap_Bootsplash_copy[(LCD_WIDTH * 2)],
					LCD_WIDTH);
			memcpy(&image[3][0], &epd_bitmap_Bootsplash_copy[(LCD_WIDTH * 3)],
					LCD_WIDTH);
			//memcpy(&image[4][0],&epd_bitmap_Bootsplash_copy[512], 128);
		}

		// flash image to LCD
		osKernelLock();
		LCDWriteBitmap((uint8_t*) &image[0]);
		osKernelUnlock();

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartserialTask */
/**
 * @brief Function implementing the serialTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartserialTask */
void StartserialTask(void *argument)
{
	/* USER CODE BEGIN StartserialTask */

	HAL_UART_Receive_DMA(&huart1, (uint8_t*) &UART1_RX_Buffer,
			sizeof(UART1_RX_Buffer));

	int loopcount = 0;
	int8_t debounce[32] =
	{ 0 };
	KDU_KeyPress_t _lastkeypress =
	{ 0 };
	KDU_KeyPress_t lastreading =
	{ 0 };
	KDU_KeyData_t outputdata =
	{ 0 };

	snprintf(outputdata.BuildDate, sizeof(outputdata.BuildDate), "%s",
			__DATE__);
	snprintf(outputdata.BuildTime, sizeof(outputdata.BuildTime), "%s",
			__TIME__);

	uint32_t LastRXTicks = osKernelGetTickCount();
	uint32_t LastRXTicksDelta = osKernelGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		loopcount++;
		osDelay(5);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		// read keypad matrix
		lastreading.regval = 0;
		ScanKeypad(&lastreading);

		bool keychange = false;

		// perform debounce, we keep a counter per button that is set high
		// when down, and counts down to 0
		_lastkeypress.regval = CurrentKey.regval;
		CurrentKey.regval = 0;
		for (int i = 0; i < 32; i++)
		{
			bool bitset = (lastreading.regval & 1 << i) > 0;
			if (bitset)
			{
				debounce[i] = KeypadDebounce;
			}
			else if (debounce[i] > 0)
			{
				debounce[i]--;
			}

			if (debounce[i] > 0)
				CurrentKey.regval |= 1 << i;
		}

		// check if we need to send an update
		if (_lastkeypress.regval != CurrentKey.regval)
			keychange = true;

		// output data if a key changed, or approximately every 500 ms
		if (keychange || loopcount % 32 == 0)
		{
			// generate UART TX packet

			// wait for previous transmission to end if still running
			while (huart1.gState != HAL_UART_STATE_READY)
				osDelay(1);

			// load some parameters
			outputdata.KeyMask = CurrentKey;
			outputdata.SequenceCounter++;
			outputdata.TickCount = osKernelGetTickCount();

			// make a bitmask with all keys pressed this sequence
			// this should make us output one ASCII character per key down event
			KDU_KeyPress_t deltapress;
			deltapress.regval = CurrentKey.regval ^ _lastkeypress.regval;
			deltapress.regval &= CurrentKey.regval;

			outputdata.ASCIIKey = KDUKeyPressToASCIIByte(deltapress);

			// load more standard parameters
			outputdata.Backlight = PWM;
			outputdata.Contrast = bias;
			outputdata.ProgrammedContrast = writefram.LCD_Contrast;
			outputdata.SerialNo = writefram.SerialNo;
			outputdata.ResetReason = rstcause;
			outputdata.TicksSinceRX = MIN(
					MAX(osKernelGetTickCount() - LastRXTicks, LastRXTicksDelta),
					UINT16_MAX);

			// signal if the FRAM was ok on startup in bit 0, and bit 1 is high if a write was successfully attempted
			outputdata.FRAMStatus = poweronframvalid | writeframok ? 2 : 0;

			int count = SerializeArray(3, 1, sizeof(KDU_KeyData_t),
					(char*) &outputdata, &UART1TXBuffer[0],
					sizeof(UART1TXBuffer), false);

			HAL_UART_Transmit_DMA(&huart1, (const uint8_t*) UART1TXBuffer,
					count);
		}

		// check for RX errors and reset if so
		bool rxne = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
		bool fe = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE);
		if (rxne || fe)
		{
			__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
			__HAL_UART_CLEAR_FEFLAG(&huart1);
			HAL_UART_AbortReceive(&huart1);
			//memset(&UART1_RX_Buffer, 0, sizeof(UART1_RX_Buffer));
			HAL_UART_Receive_DMA(&huart1, (uint8_t*) &UART1_RX_Buffer,
					sizeof(UART1_RX_Buffer));
			USART1_DMA_New_Data(true);
		}

		// write the F-RAM two times, hopefully guaranteeing that one will always be readable
		if (validdisplay->WriteFRAM == 1)
		{
			writefram.LCD_Contrast = bias;
			validdisplay->WriteFRAM = false;
			writeframok = true;
			writeframok &= WriteFRAM(0, 0, &writefram);
			writeframok &= WriteFRAM(0, 1, &writefram);
		}

		// check for new UART DMA data and process
		USART1_DMA_New_Data(false);

		if (ReceivedDisplayData)
		{
			LastRXTicksDelta = osKernelGetTickCount() - LastRXTicks;
			LastRXTicks = osKernelGetTickCount();
			ReceivedDisplayData = false;
		}
	}
	/* USER CODE END StartserialTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM8 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM8)
	{
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
