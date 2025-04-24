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
#include "pdm2pcm.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "cs43l22.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int waveType;
	float frequency;
	float cyclesPerSample;
	double phase;
} wave;

typedef struct {
	int complexity;
	wave waves[10];
	int volume; //this should be treated as a percent
	float ampRamp;
	int duration;
	int startTime;
} sound;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DAC_ADDRESS (0x94)
#define numberRangePerSign 32767 //see if you can figure out a way to change bit depth
#define sigLength 256
#define beatsPerMeasure 8
#define sine_t 1
#define noise_t 2
#define triangle_t 3
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
bool DAC_reset (void);
void advanceSine (wave *inQuestion);
int16_t sineValue (wave inQuestion);
void topUp (int startAt, int endAt);
void DebugCheckpoint ();
void drum ();
bool is_same_sound (sound first, sound second);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t sampleRate = AUDIO_FREQUENCY_22K;
wave* waves;
int activeSounds = 0;
sound sounds [10];

const sound lowDrum = {
    .complexity = 3,
    .waves      = { { triangle_t, 200, 0.00907029, 0.0 }, { triangle_t, 205, 0.00929705, 0.0 }, { noise_t, 0, 0, 0.0 } },  /* rest 0 */
    .volume     = 10,
    .ampRamp    = -20.0f,
    .duration   = 0,
    .startTime  = 0
};
const sound rest = {
	.complexity = 0,
	.waves      = {{0}},  /* rest 0 */
	.volume     = 0,
	.ampRamp    = 0.0f,
	.duration   = 0,
	.startTime  = 0
};

//sineWave hundredPerSample = {366.211, 0.0f};
wave wave1 = {sine_t, 200};
wave wave2 = {sine_t, 400};
wave wave3 = {noise_t, 500};
wave wave4 = {sine_t, 500};
wave wave5 = {sine_t, 500};
wave wave6 = {sine_t, 500};

sound beat [8] = {lowDrum, rest, lowDrum, rest, lowDrum, rest, lowDrum, rest};

int16_t __attribute__((aligned(4))) sig[sigLength];
int roughMilliseconds = 0;
int beatBaseTime = 0;
int measureBaseTime = 0;
float secondsPerBeat = 0.5f;

//bool orderFlag = true;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */
  AUDIO_IO_Init();
  cs43l22_Init(DAC_ADDRESS, OUTPUT_DEVICE_AUTO, 85, sampleRate);
  cs43l22_SetVolume(DAC_ADDRESS, 60);
  cs43l22_Play(DAC_ADDRESS, (uint16_t*) sig, sigLength);
  topUp(0, sigLength-1);
  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) sig, sigLength);
//  int i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
    /* USER CODE BEGIN 3 */
    if (roughMilliseconds > beatBaseTime + secondsPerBeat * 1000) {
    	if (roughMilliseconds > measureBaseTime + secondsPerBeat * 8 * 1000) {
    		measureBaseTime = roughMilliseconds;
    	}
    	beatBaseTime = roughMilliseconds;
    	drum();
    }
//	if (sounds[0].complexity > 1 || sounds[0].complexity < 0) {
//		Error_Handler();
//	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */
  hi2s3.Init.AudioFreq = sampleRate;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

bool DAC_reset (void) {
	HAL_I2C_DeInit(&hi2c1);
	HAL_Delay(10);
	HAL_I2C_Init(&hi2c1);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_StatusTypeDef I2Cstatus = HAL_I2C_IsDeviceReady(&hi2c1, DAC_ADDRESS, 1, 100);
	if (I2Cstatus != HAL_OK) {
		return false;
	}
	uint16_t zeroSample = 0;
	HAL_StatusTypeDef I2Sstatus = HAL_I2S_Transmit(&hi2s3, &zeroSample, 1, 100);
	if (I2Sstatus != HAL_OK) {
		return false;
	}
	return true;
}

void advanceSine (wave *inQuestion) {
//	inQuestion->phase = fmod(sampleCount * inQuestion->cyclesPerSample * (M_PI * 2), M_PI * 2);
	inQuestion->phase += inQuestion->cyclesPerSample * (M_PI * 2);
	inQuestion->phase = fmod(inQuestion->phase, M_PI * 2);
}

void advanceTriangle (wave *inQuestion) {
	inQuestion->phase += inQuestion->cyclesPerSample;
	inQuestion->phase = fmod(inQuestion->phase, 1.0f);
}

int16_t sineValue (wave inQuestion) {
	return sinf(inQuestion.phase) * numberRangePerSign;
}

int16_t triangleValue (wave inQuestion) {
	float a = fabs(inQuestion.phase - 0.5);
	float b = a - 0.25;
	int16_t c = b * numberRangePerSign * 4;
	return c;
}

int16_t waveSampleValue (wave *inQuestion) {
	int toReturn = 0;
	switch (inQuestion->waveType) {
		case sine_t:
			toReturn = sinf(inQuestion->phase) * numberRangePerSign;
			advanceSine(inQuestion);
			break;
		case noise_t:
			toReturn = (rand() % numberRangePerSign) - numberRangePerSign * 0.5f;
			break;
		case triangle_t:
			toReturn = (fabs(inQuestion->phase - 0.5) - 0.25) * numberRangePerSign * 4;
			advanceTriangle(inQuestion);
			break;
	}
	return toReturn;
}

void topUp (int startAt, int endAt) {
	for (int i = startAt; i <= endAt; i += 2) {
		sig[i] = 0;
		for (int soundListIndex = 0; soundListIndex < activeSounds; soundListIndex++) {
			for (int waveIndex = 0; waveIndex < sounds[soundListIndex].complexity; waveIndex++) {
//	if (waveIndex > 1) {
//		Error_Handler();
//	}
	//			advanceSine(&waves[soundListIndex]);
				float secondsPassed = (roughMilliseconds - sounds[soundListIndex].startTime) * 0.001;
				float gain = 0.01 * (sounds[soundListIndex].volume + sounds[soundListIndex].ampRamp * secondsPassed);
				sig[i] += waveSampleValue(&sounds[soundListIndex].waves[waveIndex]) * gain;
			}
		}
		sig[i + 1] = sig[i];
	}
	roughMilliseconds += (float)sigLength / (float)sampleRate * 500; //Remember that this is called every HALF buffer.
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
//	if (orderFlag == false) {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		topUp(sigLength / 2, sigLength - 1);
	    __DMB();
//		orderFlag = true;
//	}
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
//	if (orderFlag == true) {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		topUp(0, sigLength / 2 - 1);
	    __DMB();
//		orderFlag = false;
//	}
}

void DebugCheckpoint () {
	  while (0)	  {
	  }
}

void drum()  {
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	int currentBeat = (int) ((roughMilliseconds - measureBaseTime) / (secondsPerBeat * 8 * 1000) * 8);
	if (is_same_sound(sounds[0], rest) == false && is_same_sound(beat[currentBeat], rest)) {
		activeSounds -= 1;
	}
	if (is_same_sound(sounds[0], rest) && is_same_sound(beat[currentBeat], rest) == false) {
		activeSounds += 1;
	}
	sounds[0] = beat[currentBeat];
	sounds[0].startTime = roughMilliseconds;
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

bool is_same_sound (sound first, sound second) {
	if (first.ampRamp == second.ampRamp && first.complexity == second.complexity && first.duration == second.duration && first.waves[0].waveType == second.waves[0].waveType && first.waves[0].frequency == second.waves[0].frequency){
		return true;
	}
	return false;
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
