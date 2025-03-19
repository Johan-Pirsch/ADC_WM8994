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
#include <stdio.h>
#include "wm8994.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SW_RESET		0x0000

#define BUFFER_SIZE		1024
#define WM8994_ADDRESS  0x0034
#define ANTI_POP_2		0x0039
#define ANTI_POP_1		0x0038

#define PWR_MANAGE_1	0x0001
#define PWR_MANAGE_5	0x0005
#define DAC1_LEFT_MIXR	0x0601
#define DAC1_RIGHT_MIXR	0x0602
#define DAC2_LEFT_MIXR	0x0604
#define DAC2_RIGHT_MIXR	0x0605

#define IN_MIXR_1		0x0015
#define IN_MIXR_2		0x0028
#define IN_MIXR_3		0x0029
#define IN_MIXR_4		0x002A
#define IN_MIXR_5 		0x002B
#define IN_MIXR_6		0x002C

#define PWR_MANAGE_4	0x0004
#define AIF1_DRC_1		0x0440
#define PWR_MANAGE_2	0x0002
#define AIF1_LEFT_MIXR	0x0606
#define AIF1_RIGHT_MIXR	0x0607
#define GPIO1_INT_1		0x0700

#define AIF1_RATE		0x0210
#define AIF1_CONTROL	0x0300
#define MASTER_SLAVE	0x0302

#define DSP_CLOCK		0x0208
#define AIF1_CLOCK		0x0200

#define OUT_MIXR_1		0x002D
#define OUT_MIXR_2		0x002E
#define OUT_MIXR_3		0x002F
#define OUT_MIXR_4		0x0030
#define OUT_MIXR_5		0x0031
#define OUT_MIXR_6		0x0032

#define WRITE_SEQ_CTRL_1	0x0110

#define AIF1_DAC1_FILTERS_1	0x0420

#define CLASS_W_1		0x0051
#define ANALOG_HP_1		0x0060
#define CHARGE_PUMP_1	0x004C
#define CHARGE_PUMP_2	0x004D
#define PWR_MANAGE_3	0x0003
#define DC_SERVO_1		0x0054

#define DAC1_LEFT_VOL	0x0610
#define DAC1_RIGHT_VOL	0x0611
#define DAC2_LEFT_VOL	0x0612
#define DAC2_RIGHT_VOL	0x0613

#define AIF1_DAC2_FILTERS 0x0422

#define LEFT_LINE_IN_1_2_VOL	0x0018
#define	RIGHT_LINE_IN_1_2_VOL 	0x001A
#define AIF1_ADC1_FILTER		0x0410


#define RIGHT_OUT_VOL 			0x001D
#define LEFT_OUT_VOL			0x001C

#define SAI_BUFFER_SIZE			1024
#define SPEAKER_MIXR_L			0x0036
#define SPEAKER_MIXR_R			0x0036

#define SPEAKER_MIXR_ATN_L		0x0022
#define SPEAKER_MIXR_ATN_R		0x0023

#define SPEAKER_VOL_L			0x0026
#define SPEAKER_VOL_R			0x0027

#define SPKOUT_MIXR				0x0024
#define SPEAKER_STARTUP			0x0110

#define INPUT_MIXR_1			0x0015
#define HEADPHONE_STARTUP		0x0110


#define LEFT_OPGA_VOL 			0x0020
#define RIGHT_OPGA_VOL 			0x0021


#define AIF1_CLOCKING_1 		0x0200
#define AIF1_CLOCKING_2			0x0201

#define AIF1_CLK_RATE			0x0210

#define AIF1_CNTRL_1			0x0300
#define AIF1_MASTER_SLAVE 		0x0302

#define CLK_1_RATE				0x0208
#define CLK_2_RATE				0x0209

#define AIF1_ADC1L_AIF1MIXR		0x0606
#define AIF1_ADC1R_AIF1MIXR		0x0607

#define AIF1_ADC1_LEFT_VOL		0x0400
#define AIF1_ADC1_RIGHT_VOL		0x0401
#define AIF1_DAC1_LEFT_VOL 		0x0402
#define AIF1_DAC1_RIGHT_VOL 	0x0403

#define AIF1_DAC11_LEFT_VOL		0x0402
#define AIF1_DAC11_RIGHT_VOL	0x0403

#define AIF1_DAC1_FILTERS_1		0x0420

#define DAC_SOFTMUTE			0x0614

#define GPIO_1_CONTROL			0x0700
#define AIF1_BCLK_1				0x0303
#define AIF1_ADC_LRCLK			0x0304
#define AIF1_DAC_LRCLK			0x0305
#define AIF1_CNTRL_2			0x0301
#define AIF1_DAC_DATA			0x0306
#define AIF1_ADC_DATA			0x0307

#define  SPKMIXL_ATTN			0x0022
#define  SPKMIXR_ATTN			0x0023

#define  CLASS_D				0x0025


#define AIF1_ADC1_FILTERS		0x0410

#define DAC1_MIX_VOL			0x0600
#define SIDE_TONE				0x0621
#define	CLOCKING_1				0x0208
#define	CLOCKING_2				0x0209
#define OVERSAMPLING			0x0620

#define DAC1L_MIXR_ROUTING		0x0601
#define DAC1R_MIXR_ROUTING		0x0602
#define SPKR_MIXR				0x0034

#define CNTRL_SEQ_SPEAKER		0x0110
#define AIF1_BCLK				0x0303
#define AIF1ADC_LRCLK			0x0304
#define AIF1DAC_LRCLK			0x0305

#define AIF2_DACL_LEFT_VOL		0x0502
#define AIF2_DACR_LEFT_VOL		0x0503
#define AIF2_DAC_FILTERS_1		0x0520

#define AIF2_CLOCKING_1			0x0204
#define AIF1_DAC_FILTERS_1		0x0420


#define AUDIO_BUFFER_SIZE 4096  // Circular buffer size
#define DMA_BUFFER_SIZE 1024    // Half of the circular buffer for DMA transfers


#define RECORD_BUFFER_SIZE  4096

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

I2S_HandleTypeDef hi2s1;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

static uint32_t frequency = AUDIO_FREQUENCY_48K;
static uint8_t volume = 80;


typedef enum
{
    BUFFER_OFFSET_NONE = 0,
    BUFFER_OFFSET_HALF = 1,
    BUFFER_OFFSET_FULL = 2,
} BUFFER_StateTypeDef;

volatile uint32_t audio_rec_buffer_state;
volatile uint32_t audio_tx_buffer_state = 0;


int16_t RecordBuffer[RECORD_BUFFER_SIZE];

int16_t PlaybackBuffer[RECORD_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SAI1_Init(void);
static void MX_I2C4_Init(void);
static void MX_I2S1_Init(void);
/* USER CODE BEGIN PFP */


static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint16_t BufferSize);
void get_max_val(int16_t *buf, uint32_t size, int16_t amp[]);
void AUDIO_LOOPBACK(void);


void WM_8994_ADC_SPKR(void);
void wm_8994_read_reg(void);
void wm_8994_ADC(void);
int16_t shift_register(uint16_t data);
void WM_8994_AIF1_SPKR(void);



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SAI1_Init();
  MX_I2C4_Init();
  MX_I2S1_Init();
  /* USER CODE BEGIN 2 */
  WM_8994_AIF1_SPKR();


 //WM_8994_ADC_SPKR();
 AUDIO_LOOPBACK();
  /* USER CODE END 2 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00606092;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s1.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s1.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s1.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

	/* Disable SAI peripheral to allow access to SAI internal registers */
   __HAL_SAI_DISABLE(&hsai_BlockA1);

   __HAL_RCC_SAI1_CLK_ENABLE();


  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.Mckdiv         = 0;



  /* Configure SAI_Block_x Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  hsai_BlockA1.FrameInit.FrameLength = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;


  /* Configure SAI Block_x Slot
    Slot First Bit Offset: 0
    Slot Size  : 16
    Slot Number: 4
    Slot Active: All slot actives */
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 4;
  hsai_BlockA1.SlotInit.SlotActive = SAI_SLOTACTIVE_0 |SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2| SAI_SLOTACTIVE_3;;

  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_FREE_PROTOCOL, SAI_PROTOCOL_DATASIZE_16BIT, 4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SAI_Init(&hsai_BlockA1);

   /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&hsai_BlockA1);


  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&hsai_BlockB1);

  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.Protocol  = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.DataSize  = SAI_DATASIZE_16;
  hsai_BlockB1.Init.FirstBit  = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_RELEASED;

  /* Configure SAI_Block_x Frame */
  hsai_BlockB1.FrameInit.FrameLength       = 64;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB1.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockB1.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

     /* Configure SAI Block_x Slot */
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber     = 4;
  hsai_BlockB1.SlotInit.SlotActive     = SAI_SLOTACTIVE_0 |SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2| SAI_SLOTACTIVE_3;;

  HAL_SAI_Init(&hsai_BlockB1);

     /* Enable SAI peripheral */
  __HAL_SAI_ENABLE(&hsai_BlockB1);

  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_FREE_PROTOCOL, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */


 	 // Enable DMA for SAI A and SAI B
 	 HAL_DMA_Init(&hdma_sai1_a);
 	 HAL_DMA_Init(&hdma_sai1_b);

 	    // Link DMA to SAI for both A and B
 	 __HAL_LINKDMA(&hsai_BlockA1, hdmatx, hdma_sai1_a);
 	 __HAL_LINKDMA(&hsai_BlockB1, hdmarx, hdma_sai1_b);

 	 HAL_DMA_DeInit(&hdma_sai1_b);

 	 HAL_DMA_DeInit(&hdma_sai1_a);

 	 HAL_DMA_Init(&hdma_sai1_b);

 	 HAL_DMA_Init(&hdma_sai1_a);

  /* USER CODE END SAI1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */

	__HAL_DMA_DISABLE(&hdma_sai1_a);
    __HAL_RCC_DMA2_CLK_ENABLE();

		  /* Peripheral DMA init*/

	hdma_sai1_a.Instance = DMA2_Stream1;
	hdma_sai1_a.Init.Request = DMA_REQUEST_SAI1_A;
	hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
	hdma_sai1_a.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	__HAL_DMA_ENABLE(&hdma_sai1_a);
	if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK)
		{
		   Error_Handler();
		}

		/* Several peripheral DMA handle pointers point to the same DMA handle.
		 Be aware that there is only one channel to perform all the requested DMAs. */


  __HAL_LINKDMA(&hsai_BlockA1,hdmatx,hdma_sai1_a);


  __HAL_DMA_DISABLE(&hdma_sai1_b);


		  /* Peripheral DMA init*/

	hdma_sai1_b.Instance = DMA2_Stream4;
	hdma_sai1_b.Init.Request = DMA_REQUEST_SAI1_B;
    hdma_sai1_b.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sai1_b.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sai1_b.Init.MemInc = DMA_MINC_ENABLE;
	hdma_sai1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_sai1_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_sai1_b.Init.Mode = DMA_CIRCULAR;
	hdma_sai1_b.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_sai1_b.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	__HAL_DMA_ENABLE(&hdma_sai1_b);
	if (HAL_DMA_Init(&hdma_sai1_b) != HAL_OK)
		{
		  		  Error_Handler();
		 }

		/* Several peripheral DMA handle pointers point to the same DMA handle.
		 Be aware that there is only one channel to perform all the requested DMAs. */
	__HAL_LINKDMA(&hsai_BlockB1,hdmarx,hdma_sai1_b);



		  /* DMA interrupt init */
		  /* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		  /* DMA2_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CEC_CK_MCO1_Pin */
  GPIO_InitStruct.Pin = CEC_CK_MCO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(CEC_CK_MCO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* CODEC_SAI pins configurations: FS, SCK, and SD Pins */
  /*ENABLE FS, SCK AND SD PINS */
  GPIO_InitStruct.Pin = SAI1_SD_B_Pin| SAI1_FS_A_Pin | SAI1_SCK_A_Pin | SAI1_SD_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  /*ENABLE MCLK CLOCK */
  GPIO_InitStruct.Pin = SAI1_MCLK_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/**
  * @brief  AUDIO Codec delay
  * @param  Delay: Delay in ms
  */

// DMA Callback for handling SAI1 TX



void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	audio_tx_buffer_state = 1;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
	audio_rec_buffer_state = BUFFER_OFFSET_FULL;

}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	audio_rec_buffer_state = BUFFER_OFFSET_HALF;
}

void AUDIO_LOOPBACK(void) {
	 int16_t amp[4];
	 int32_t ret;

	 ret = HAL_SAI_Receive_DMA(&hsai_BlockB1,(uint8_t*)RecordBuffer, RECORD_BUFFER_SIZE);
	 if(ret == HAL_OK) {
	 		printf("SAI receive begin OK\n");
	 		}
	 else {
		 printf("SAI receive error: %d\r\n", ret);
	 }

	 printf("Copying Record buffer to Playback buffer\r\n");

	if(HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*) PlaybackBuffer, RECORD_BUFFER_SIZE) == HAL_OK)

		{
			printf("Audio output OK\r\n");
			HAL_SAI_Transmit_DMA(&hsai_BlockA1, &PlaybackBuffer[0], RECORD_BUFFER_SIZE);

		}

	else {
		 printf("Audio output error\r\n");
	}

	audio_rec_buffer_state = BUFFER_OFFSET_NONE;

	while (1) {

	     /* 1st or 2nd half of the record buffer ready for being copied
	     to the Playback buffer */
	     if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
	             {
	              /* Copy half of the record buffer to the playback buffer */
	              if (audio_rec_buffer_state == BUFFER_OFFSET_HALF)
	                 {
	                     get_max_val(RecordBuffer, RECORD_BUFFER_SIZE / 2, amp);
	                     CopyBuffer(&PlaybackBuffer[0], &RecordBuffer[0], RECORD_BUFFER_SIZE / 2);
	                 } else {
	                	 /* if(audio_rec_buffer_state == BUFFER_OFFSET_FULL)*/
	                     CopyBuffer(&PlaybackBuffer[RECORD_BUFFER_SIZE / 2],
	                    		      &RecordBuffer[RECORD_BUFFER_SIZE / 2],
									                RECORD_BUFFER_SIZE / 2);
	                 }
	                 /* Wait for next data */
	                 audio_rec_buffer_state = BUFFER_OFFSET_NONE;
	             }
	             if (audio_tx_buffer_state)
	             {
	                 audio_tx_buffer_state = 0;
	             }
	         } // end while(1)

}


/* get maximum value of the buffer. VU meter, perhaps?
*/
void get_max_val(int16_t *buf, uint32_t size, int16_t amp[])
{
   int16_t maxval[4] = { -32768, -32768, -32768, -32768};
   uint32_t idx;
   for (idx = 0 ; idx < size ; idx += 4) {
       if (buf[idx] > maxval[0])
           maxval[0] = buf[idx];
       if (buf[idx + 1] > maxval[1])
           maxval[1] = buf[idx + 1];
       if (buf[idx + 2] > maxval[2])
           maxval[2] = buf[idx + 2];
       if (buf[idx + 3] > maxval[3])
           maxval[3] = buf[idx + 3];
   }
   memcpy(amp, maxval, sizeof(maxval));
}


static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint16_t BufferSize)
{
    uint32_t i = 0;
    for (i = 0; i < BufferSize; i++)
    {
        pbuffer1[i] = pbuffer2[i];
    }
}




void wm_8994_read_reg(void){

	int32_t ret;
	uint16_t tmp[3];
	uint16_t data;


	ret = HAL_I2C_IsDeviceReady(&hi2c4, WM8994_ADDRESS, 2, 1);

	if(ret == HAL_OK) {
		printf("Communication with device received\n");
		HAL_Delay(500);

		ret = HAL_I2C_Mem_Read(&hi2c4, WM8994_ADDRESS, SW_RESET, I2C_MEMADD_SIZE_16BIT, &data, 2, 1);

		if(ret >= 0) {
			tmp [0] = ((uint16_t)(data >> 8) & 0x00FF);
			tmp [1] = ((uint16_t)(data << 8) & 0xFF00);

			tmp[2] = tmp[0] + tmp[1];

			printf("Device ID: %04x\n", tmp[2]);
			HAL_Delay(500);
		}
		else {
			printf("No Device ID Found\n");
		}
	}

	else {
		printf("No communication was established\n");
		HAL_Delay(500);
	}

}

int16_t shift_register(uint16_t data) {
	uint16_t passed_byte;
	uint16_t buf_data[3];
	int32_t ret;
	ret = HAL_I2C_IsDeviceReady(&hi2c4, WM8994_ADDRESS, 2, 1);


	if(ret == HAL_OK) {
		printf("Communication with device received\n");
		HAL_Delay(50);

		buf_data [0] = ((uint16_t)(data >> 8) & 0x00FF);
		buf_data [1] = ((uint16_t)(data << 8) & 0xFF00);

		buf_data[2] = buf_data[0] + buf_data[1];
		passed_byte = buf_data[2];

			//ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SW_RESET, I2C_MEMADD_SIZE_16BIT, &data, 2, 1);


		printf("Sent Data: %04x\n", passed_byte);
		HAL_Delay(500);
		return passed_byte;
	}

	else {
		printf("No communication was established\n");
		HAL_Delay(500);
		return 0;
	}



}


void WM_8994_AIF1_SPKR() {

	uint16_t tx_bit;
	uint16_t rx_bit;
	int32_t ret;
	/*SPEAKER SARTUP SEQUENCE*/
	tx_bit = 0x8110;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CNTRL_SEQ_SPEAKER , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	HAL_Delay(50);

	/* wm8994 Errata Work-Arounds */
	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, 0x102 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,  0x817 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, 0x102 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*ENABLE VMID SOFT START FAST, STARTUP BIAS CURRENT ENABLED */

	tx_bit = 0x002C; /*VMID RAMP (01) NORMAL FAST START, VMID BUFFER ENABLE (1), START UP BIAS ENABLED (1), BIAS_SRC (0) NORMAL BIAS, VMID DISCH (0) */
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, ANTI_POP_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
	/*CHECKED 3-12-25*/

	/*MICB1_ENA (0), VMID_SEL (01), BIAS_ENA (1), REST OF THE BITS ARE ZERO */

	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_1 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
	/*CHECKED 3-12-25

	/* ADD A DELAY */

	HAL_Delay(50);

	/*INPUT PGA ENABLE*/
	/* PWR MANAGEMENT (2) */
	/* REGISTER 2 SETTINGS */
	/* OPCLK_ENA (0) GPIO CLOCK OUTPUT DISABLED  ,MIXINL_ENA (1) ENABLED, MIXINR_ENA (1), ENABLED, IN2L_ENA (0) DISABLED, IN1L_ENA (1) ENABLED, IN2R_ENA (0) DISABLED, IN1R_ENA (1) ENABLED.
	*/
	/*VERFIED 3-12-25*/

	tx_bit = 0x0350;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*INPUT PGA CONFIGURATION*/
	/* INPUT MIXER (2) */
	/* REGISTER 40 SETTINGS */
	/*IN1RN CONNECTED TO IN1R 1, IN1RP_TO_IN1R CONNECTED TO VMID 0, IN2RN TO IN2R 0, IN2RP TO IN2R CONNECTED TO VMID 0
	* IN1LN TO IN1L 1, IN1LP TO IN1L CONNECTOED TO VMID 0, IN2LN TO IN2L 0, IN2LP TO IN2L 0, REST BITS 0
	*/
	/*CHECKED 3-12-25*/

	tx_bit = 0x00FF;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, IN_MIXR_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*LEFT LINE INPUT 1 & 2 VOLUME REGISTER R24 */
	/* IN1_VU (1), IN1L_MUTE (0), IN1L_ZC (0), IN1L_VOL (01110) +4.5 DB */
	/*MEASURED OUTPUT OF PC VOLUME ON HEADPHONES WAS ABOUT 500 mV GAIN OF 4.5 DB IS ABOUT 900 mV*/
	/*VERIFIED 3-12-25*/

	tx_bit = 0x010E; //4.5db
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, LEFT_LINE_IN_1_2_VOL, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*SKIPPING LEFT LINES 3 AND 4 */

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, RIGHT_LINE_IN_1_2_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT MIXER ENABLE */
	/* R2 PWR MANAGEMENT 2 */
	/* REGISTER 2 SETTINGS */
	/* OPCLK_ENA (0),MIXINL_ENA (1) ENABLED, MIXINR_ENA (1), ENABLED, IN2L_ENA (0) DISABLED, IN1L_ENA (1) ENABLED, IN2R_ENA (0) DISABLED, IN1R_ENA (1) ENABLED.
	*/
	/*VERIFIED 3-12-25*/

	tx_bit = 0x0350;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT MIXER (3) R41 */
	/*IN1L_MIXINL (1) UNMUTE, IN1L_MIXINL_VOL (0) 0 DB, MIXOUTL_MIXINL_VOL (000) MUTE, NEED TO MUTE RECORD PATH*/
	/*VERIFIED 3-12-25*/
	tx_bit = 0x0020; //RECORD PATH SET TO MUTE (000),  IN1L PGA OUTPUTT TO MIXINL GAIN (0) 0 DB
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, IN_MIXR_3, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT MIXER (4) R42 */
	/*IN1R_MIXINR (1) UNMUTE, IN1R_MIXINR_VOL (0) 0 DB, MIXOUTR_MIXINR_VOL (000) MUTE, NEED TO MUTE RECORD PATH*/
	/*VERIFIED 3-12-25*/

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, IN_MIXR_4, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/* AIF1 CLOCKING (1) R512 (0x0200) */
	/*AIF1_CLK_SRC (00) MCLK1, AIF1CLK_INV (0) AIF1CLK NOT INVERTED, AIF1CLK_DIV (0) AIF1CLK, AIF1CLK_ENA (1) ENABLED */
	tx_bit = 0x0001;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CLOCKING_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*AIF1_CLOCKING_2 (0x0201) R513 */
	/*AIF1DAC_DIV (000) DIVIDE BY 1 */
	/*AIF1ADC_DIV (000) DIVIDE BY 1 */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CLOCKING_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*AIF1 RATE R528 (0x0210) */
	/*SELECT THE AIF1 SAMPLE RATE (FS) */
	/*AIF1 SAMPLE RATE (FS) 48 KHZ (1000) */
	/*AIF1 CLK RATE (0011) 256 */
	tx_bit = 0x0083;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_RATE, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT PGA ENABLE*/
	/* PWR MANAGEMENT (2) */
	/* REGISTER 2 SETTINGS */
	/* OPCLK_ENA (0) GPIO CLOCK OUTPUT DISABLED  ,MIXINL_ENA (1) ENABLED, MIXINR_ENA (1), ENABLED, IN2L_ENA (0) DISABLED, IN1L_ENA (1) ENABLED, IN2R_ENA (0) DISABLED, IN1R_ENA (1) ENABLED.
	*/
	/*VERFIED 3-16-25*/

	/*GPIO OUTPUT CLK ENABLE */
	/*CONTROLLED BY R2 POWER MANAGEMENT */
	tx_bit = 0x0350;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*GPIO1 PIN DIRECTION */
	/*GP1_DIR (0) OUTPUT */
	/*GPI_PU (0) DISABLED, GP1_PD (0) DISABLED, GP1_POL (0) NON-INVERTED ACTIVE HIGH, GP1_OP_CFG (0) CMOS */
	/*GP1_LVL (1), GP1_FN (00011) (INTERRUPT IRQ OUTPUT) */

	tx_bit = 0x0143;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,GPIO_1_CONTROL, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);




	/*R520 (0x208) CLOCKING_1 */
	/* TOCLK_ENA (0) DISABLED, AIF1DSPCLK_ENA (1) ENABLED, AIF2DSPCLK_ENA (1), SYSDSPCLK_ENA (1), SYSCLK_SRC (0) AIF1CLK */

	tx_bit = 0x000E;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CLOCKING_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*R521 CLOCKING_2 (0x0209) */
	/*TOCLK_DIV [2:0] (000), DBCLK_DIV [2:0] (000), OPCLK_DIV (000) */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CLOCKING_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/* R1568 (0x0620) OVERSAMPLING */
	/* ADC_OSR ADC/DIGITAL MICROPHONE OVERSAMPLE RATE SELECT (1), DAC_OSR128 (1) HIGH PERFORMANCE */
	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, OVERSAMPLING, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*Af1 MASTER/SLAVE AND TRI STATE CONTROL*/
	/*R770 (0x0302) AIF1 MASTER/SLAVE */
	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, MASTER_SLAVE, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*ANALOG TO DIGITAL CONVERTER (ADC ENABLE CONTROL REGISTER R4 (0X0004) ) */
	/* AIF1ADC1L_ENA (1) ENABLED, AIF1ADC1R_ENA (1) ENABLED, ADCL_ENA LEFT ADC ENABLE (1), ADCR_ENA RIGHT ADC ENABLE (1)*/
	/*REST OF THE BITS ARE ZERO */
	/*VERIFIED 3-12-25*/
	tx_bit = 0x0303;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_4 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);



	/*R5 POWER MANAGEMENT (0x0005) */
	/*AIF1DAC1L_ENA (1) ENABLE AIF1DAC11 (LEFT INPUT PATH) AIF1 TIME SLOT 0 */
	/*ENABLE AIF1DAC1 (RIGHT) INPUT PATH (AIF1, TIMESLOT 0) */

	tx_bit = 0x0303;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_5 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*R768 (0x0300) AIF1 _CONTROL */
	/*AIF1_CONTROL_1*/
	/*AIF1ADCL_SRC (0), AIF1ADCR_SRC (1), AIF1ADC_TDM (0), AIF1_BLCK_INV (0), AIF1_LRCLK_INV (0), AIF1_WL[1:0] (00) 16 BITS, AIF1_FMT [1:0] (10) I2S FORMAT */
	tx_bit = 0x4010;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CNTRL_1 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*R771 (0x0303) */
	/*AIF1 BCLK*/
	/*AIF1_BCLK_DIV (00000) BCLK1 RATE = AIF1CLK */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_BCLK , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*R772 (0x0304) */
	/*AIF1ADC_LRCLK_DIR (1) ADCLRCLK1 (ENABLED IN SLAVE MODE) */
	/* AIF1ADC_RATE [10:0] (0X0040) DEFAULT VALUE */

	tx_bit = 0x0840;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1ADC_LRCLK , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);



	/*R773 (0x0305) AIF1DACLRCLK */
	/*AIF1DAC_LRCLK_DIR (1) LRCLK1 ENABLED IN SLAVE MODE */
	/* AIF1DAC_RATE [10:0] */

	tx_bit = 0x0840;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1DAC_LRCLK , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);






	/* AUDIO INTERFACE OUTPUT MIXING SECTION */
	/* REGISTER R1542 (0X0606) AIF1 ADC1 LEFT MIXR ROUTING (ADC1L_TO AIF1ADC1L (1) ENABLED) */
	/* AIF2DACL_TO_AIF1ADC1L*/
	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_ADC1L_AIF1MIXR , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*AUDIO INTERFACE 1 (AIF1) OUTPUT MIXING */
	/* REGISTER R1543 (0X0607) AIF1 ADC1 RIGHT MIXER ROUTING */
	/* ADC1R_TO_AIF1ADC1R (1) ENABLED (ENABLE ADCR/DMIC1 (RIGHT) TO AIF1 (TIMESLOT 0, RIGHT) OUTPUT) */

	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_ADC1R_AIF1MIXR , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*DAC OUTPUT DIGITAL MIXING */
	/*AIF2DACL_TO_DAC1L ENABLED */
	/*AIF2DACR_TO_DAC1R*/
	/*ENABLE AIF2 (LEFT) TO DAC1L, ENABLE AIF2 RIGHT TO DAC1R */

	tx_bit = 0x0001;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_LEFT_MIXR , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	tx_bit = 0x0001;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_RIGHT_MIXR , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);





	/* AIF1 OUTPUT PATH VOLUME CONTROL */
	/*AIF1 ADC1 LEFT VOLUME */
	/*AIF1ADC1_VU (1), AIF1ADC1L_VOL[7:0] (C0h) (0dB) */
	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_ADC1_LEFT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/* AIF1 OUTPUT PATH VOLUME CONTROL */
	/*AIF1 ADC1 RIGHT VOLUME */
	/*AIF1ADC1_VU (1), AIF1ADC1R_VOL[7:0] (C0h) (0dB) */
	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_ADC1_RIGHT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*AIF1_ADC1_FILTERS (R1040) HEX REGISTERS (0x0410) */
	/*AIF1ADC1_HPF_CUT[1:0] (00) HI-FI MODE (FC = 4 HZ AT FS = 48 KHZ) */
	/*AIF1ADC1L_HPF (1), AIF1ADC1R_HPF (1) */

	tx_bit = 0x1800;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_ADC1_FILTERS, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/* AIF1 INPUT PATH VOLUME CONTROL */
	/*AIF1 DAC1LEFT VOLUME */
	/*AIF1DAC1_VU (1), AIF1DAC1L_VOL[7:0] (C0h) (0dB) */
	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_DAC1_LEFT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/* AIF1 INPUT PATH VOLUME CONTROL */
	/*AIF1 ADC1 RIGHT VOLUME */
	/*AIF1ADC1_VU (1), AIF1ADC1R_VOL[7:0] (C0h) (0dB) */
	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_DAC1_RIGHT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*AIF2 DAC INPUT PATH SOFT MUTE CONTROL */
	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_DAC_FILTERS_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

//	/*AIF2 INPUT PATH VOLUME CONTROL */
//	/*R1282 (0X0502) */
//	/*AIF2 DACLEFT VOLUME */
//	tx_bit = 0x01C0;
//	rx_bit = shift_register(tx_bit);
//	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF2_DACL_LEFT_VOL, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
//
//	/*AIF2 INPUT PATH VOLUME CONTROL */
//	/*R1282 (0X0503) */
//	/*AIF2 DAC RIGHT  VOLUME */
//	tx_bit = 0x01C0;
//	rx_bit = shift_register(tx_bit);
//	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF2_DACR_LEFT_VOL, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
//
//	/*AIF2 DAC INPUT PATH SOFT MUTE CONTROL */
//	tx_bit = 0x0000;
//	rx_bit = shift_register(tx_bit);
//	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF2_DAC_FILTERS_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
//
//	/*AIF2 CLK ENABLE SECTION */
//
//	/* AIF2 CLOCKING (1) R512 (0x0204) */
//	/*AIF2_CLK_SRC (00) MCLK1, AIF2CLK_INV (0) AIF2CLK NOT INVERTED, AIF2CLK_DIV (0) AIF2CLK, AIF2CLK_ENA (1) ENABLED */
//	tx_bit = 0x0001;
//	rx_bit = shift_register(tx_bit);
//	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF2_CLOCKING_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
//
//


	/*R5 POWER MANAGEMENT (5) (0X0005) */
	/* AIF2DACL_ENA (0), AIF2DACR_ENA (0), AIF1DAC2L_ENA (0), AIF1DAC2R_ENA (0), AIF1DAC1L_ENA (1) ENABLED, AIF1DAC1R_ENA (1) ENABLED */
	/*DAC2L_ENA (0) DISABLED, DAC2R_ENA (0) DISABLED */
	/*DAC2L_ENA (0) DISABLED LEFT DAC2 DISABLED, DAC2R_ENA (0) DISABLED RIGHT DAC2 DISABLED */
	/*DAC1L_ENA (1) LEFT DAC 1 ENABLED, DAC1R_ENA (1) RIGHT DAC1 ENABLED */

	tx_bit = 0x0303;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_5 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*DAC DIGITAL VOLUME SECTION */
	/* DAC1 LEFT VOLUME REGISTER R1552 (0X0610) */
	/*DAC1L_MUTE (0) UN-MUTE, DAC1_VU (1) DAC1L AND DAC1R VOLUME UPDATE , DAC1L_VOL (11000000) 0 DB */

	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_LEFT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/* DAC1 RIGHT VOLUME REGISTER R1553 (0X0611) */
	/*DAC1R_MUTE (0) UN-MUTE, DAC1_VU (1) DAC1L AND DAC1R VOLUME UPDATE , DAC1R_VOL (11000000) 0 DB */

	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_RIGHT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*DAC SOFT MUTE CONTROL */
	/*DAC SOFTMUTE REGISTER R1556 (0X0614) */
	/* DAC_SOFTMUTEMODE (0) DAC UNMUTE RAMP SELECT DISABLING SOFT MUTE, DAC_MUTERATE (0) DAC SOFT MUTE RAMP RATE */
	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC_SOFTMUTE , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*OUTPUT SIGNAL PATH ENABLE */
	/*PWR MANAGEMENT REGISTER 1 R1 */

	/* SPKOUTR_ENA (1), SPKOUTL_ENA (1), HOUT2_ENA (0), HPOUT1L_ENA (0), HPOUT1R_ENA (0)  */
	/* MICB2_ENA (0), MICB1_ENA (0), 0, VMID_SEL (01), BIAS_ENA (1) */
	/*VERIFIED 3-3-25*/
	tx_bit = 0x3003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);



	/*PWR_MANAGE_3*/
	/*LINEOUT1N ENA (0), LINEOUT1P_ENA (0), LINEOUT2N ENA (0), LINEOUT2P_ENA (0), SPKRVOL ENA (1),
	* SPKLVOL_ENA (1), MIXOUTLVOL ENA (0), MIXOUTRVOL ENA (0), MIXOUTL ENA (0), MIXOUTR ENA (0),
	* REST OF THE 4 BITS ARE ZERO
	*/
	/*VERIFIED 3-3-25*/
	tx_bit = 0x0300;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_3, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*SPEAKER MIXER CONTROL SECTION
	* R54 (0X0034)
	* ENABLING LEFT SPEAKER MIXER FIRST
	* DAC2L_TO_SPKMIXL (0) MUTE, MIXINL_TO_SPKMIXL (0) MUTE, IN1LP_TO_SPKMIXL (0) MUTE, MIXOUTL_TO_SPKMIXL (0) MUTE, DAC1L_TO_SPKMIXL (1) UN-MUTE
    * */
	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SPEAKER_MIXR_L, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*R35 SPKMIXR ATTENTUATION (0X0023) */
	/* SPKOUT_CLASSAB (0) CLASS D MODE */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SPKMIXL_ATTN, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SPKMIXR_ATTN, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*SPEAKER VOLUME LEFT AND SPEAKER VOLUME RIGHT*/
	/*REGISTER R38 (0X0026) AND REGISTER R39 (0X0027)*/
	/*SPKOUT_VU (1), SPKOUTR_ZC (0), SPKOUTR_MUTE_N (1), SPKOUTR_VOL (111100)*/
	//tx_bit = 0x017C;
	//tx_bit = 0x0173;
	tx_bit = 0x017F; //+6db output volume
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,  SPEAKER_VOL_L, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,  SPEAKER_VOL_R, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);









}




void WM_8994_ADC_SPKR() {

	uint16_t tx_bit;
	uint16_t rx_bit;
	int32_t ret;
	/*SPEAKER SARTUP SEQUENCE*/
	tx_bit = 0x8110;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CNTRL_SEQ_SPEAKER , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	HAL_Delay(50);



	/*ENABLE VMID SOFT START FAST, STARTUP BIAS CURRENT ENABLED */

	tx_bit = 0x002C; /*VMID RAMP (01) NORMAL FAST START, VMID BUFFER ENABLE (1), START UP BIAS ENABLED (1), BIAS_SRC (0) NORMAL BIAS, VMID DISCH (0) */
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, ANTI_POP_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
	/*CHECKED 3-12-25*/

	/*MICB1_ENA (0), VMID_SEL (01), BIAS_ENA (1), REST OF THE BITS ARE ZERO */

	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_1 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);
	/*CHECKED 3-12-25

	/* ADD A DELAY */

	HAL_Delay(50);

	/*INPUT PGA ENABLE*/
	/* PWR MANAGEMENT (2) */
	/* REGISTER 2 SETTINGS */
	/* OPCLK_ENA (0) GPIO CLOCK OUTPUT DISABLED  ,MIXINL_ENA (1) ENABLED, MIXINR_ENA (1), ENABLED, IN2L_ENA (0) DISABLED, IN1L_ENA (1) ENABLED, IN2R_ENA (0) DISABLED, IN1R_ENA (1) ENABLED.
	*/
	/*VERFIED 3-12-25*/

	tx_bit = 0x0350;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*INPUT PGA CONFIGURATION*/
	/* INPUT MIXER (2) */
	/* REGISTER 40 SETTINGS */
	/*IN1RN CONNECTED TO IN1R 1, IN1RP_TO_IN1R CONNECTED TO VMID 0, IN2RN TO IN2R 0, IN2RP TO IN2R CONNECTED TO VMID 0
	* IN1LN TO IN1L 1, IN1LP TO IN1L CONNECTOED TO VMID 0, IN2LN TO IN2L 0, IN2LP TO IN2L 0, REST BITS 0
	*/
	/*CHECKED 3-12-25*/

	tx_bit = 0x00FF;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, IN_MIXR_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*LEFT LINE INPUT 1 & 2 VOLUME REGISTER R24 */
	/* IN1_VU (1), IN1L_MUTE (0), IN1L_ZC (0), IN1L_VOL (01110) +4.5 DB */
	/*MEASURED OUTPUT OF PC VOLUME ON HEADPHONES WAS ABOUT 500 mV GAIN OF 4.5 DB IS ABOUT 900 mV*/
	/*VERIFIED 3-12-25*/

	tx_bit = 0x010E; //4.5db
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, LEFT_LINE_IN_1_2_VOL, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*SKIPPING LEFT LINES 3 AND 4 */

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, RIGHT_LINE_IN_1_2_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT MIXER ENABLE */
	/* R2 PWR MANAGEMENT 2 */
	/* REGISTER 2 SETTINGS */
	/* OPCLK_ENA (0),MIXINL_ENA (1) ENABLED, MIXINR_ENA (1), ENABLED, IN2L_ENA (0) DISABLED, IN1L_ENA (1) ENABLED, IN2R_ENA (0) DISABLED, IN1R_ENA (1) ENABLED.
	*/
	/*VERIFIED 3-12-25*/

	tx_bit = 0x0350;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT MIXER (3) R41 */
	/*IN1L_MIXINL (1) UNMUTE, IN1L_MIXINL_VOL (0) 0 DB, MIXOUTL_MIXINL_VOL (000) MUTE, NEED TO MUTE RECORD PATH*/
	/*VERIFIED 3-12-25*/
	tx_bit = 0x0020; //RECORD PATH SET TO MUTE (000),  IN1L PGA OUTPUTT TO MIXINL GAIN (0) 0 DB
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, IN_MIXR_3, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT MIXER (4) R42 */
	/*IN1R_MIXINR (1) UNMUTE, IN1R_MIXINR_VOL (0) 0 DB, MIXOUTR_MIXINR_VOL (000) MUTE, NEED TO MUTE RECORD PATH*/
	/*VERIFIED 3-12-25*/

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, IN_MIXR_4, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/* AIF1 CLOCKING (1) R512 (0x0200) */
	/*AIF1_CLK_SRC (00) MCLK1, AIF1CLK_INV (0) AIF1CLK NOT INVERTED, AIF1CLK_DIV (0) AIF1CLK, AIF1CLK_ENA (1) ENABLED */
	tx_bit = 0x0001;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CLOCKING_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*AIF1_CLOCKING_2 (0x0201) R513 */
	/*AIF1DAC_DIV (000) DIVIDE BY 1 */
	/*AIF1ADC_DIV (000) DIVIDE BY 1 */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CLOCKING_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*AIF1 RATE R528 (0x0210) */
	/*SELECT THE AIF1 SAMPLE RATE (FS) */
	/*AIF1 SAMPLE RATE (FS) 48 KHZ (1000) */
	/*AIF1 CLK RATE (0011) 256 */
	tx_bit = 0x0083;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_RATE, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*INPUT PGA ENABLE*/
	/* PWR MANAGEMENT (2) */
	/* REGISTER 2 SETTINGS */
	/* OPCLK_ENA (0) GPIO CLOCK OUTPUT DISABLED  ,MIXINL_ENA (1) ENABLED, MIXINR_ENA (1), ENABLED, IN2L_ENA (0) DISABLED, IN1L_ENA (1) ENABLED, IN2R_ENA (0) DISABLED, IN1R_ENA (1) ENABLED.
	*/
	/*VERFIED 3-16-25*/

	/*GPIO OUTPUT CLK ENABLE */
	/*CONTROLLED BY R2 POWER MANAGEMENT */
	tx_bit = 0x0350;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*GPIO1 PIN DIRECTION */
	/*GP1_DIR (0) OUTPUT */
	/*GPI_PU (0) DISABLED, GP1_PD (0) DISABLED, GP1_POL (0) NON-INVERTED ACTIVE HIGH, GP1_OP_CFG (0) CMOS */
	/*GP1_LVL (1), GP1_FN (00011) (INTERRUPT IRQ OUTPUT) */

	tx_bit = 0x0143;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,GPIO_1_CONTROL, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);




	/*R520 (0x208) CLOCKING_1 */
	/* TOCLK_ENA (0) DISABLED, AIF1DSPCLK_ENA (1) ENABLED, AIF2DSPCLK_ENA (0), SYSDSPCLK_ENA (1), SYSCLK_SRC (0) AIF1CLK */

	tx_bit = 0x000A;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CLOCKING_1, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*R521 CLOCKING_2 (0x0209) */
	/*TOCLK_DIV [2:0] (000), DBCLK_DIV [2:0] (000), OPCLK_DIV (000) */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CLOCKING_2, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/* R1568 (0x0620) OVERSAMPLING */
	/* ADC_OSR ADC/DIGITAL MICROPHONE OVERSAMPLE RATE SELECT (1), DAC_OSR128 (1) HIGH PERFORMANCE */
	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, OVERSAMPLING, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*Af1 MASTER/SLAVE AND TRI STATE CONTROL*/
	/*R770 (0x0302) AIF1 MASTER/SLAVE */
	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, MASTER_SLAVE, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*ANALOG TO DIGITAL CONVERTER (ADC ENABLE CONTROL REGISTER R4 (0X0004) ) */
	/* AIF1ADC1L_ENA (1) ENABLED, AIF1ADC1R_ENA (1) ENABLED, ADCL_ENA LEFT ADC ENABLE (1), ADCR_ENA RIGHT ADC ENABLE (1)*/
	/*REST OF THE BITS ARE ZERO */
	/*VERIFIED 3-12-25*/
	tx_bit = 0x0303;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_4 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);



	/*R5 POWER MANAGEMENT (0x0005) */
	/*AIF1DAC1L_ENA (1) ENABLE AIF1DAC11 (LEFT INPUT PATH) AIF1 TIME SLOT 0 */
	/*ENABLE AIF1DAC1 (RIGHT) INPUT PATH (AIF1, TIMESLOT 0) */

	tx_bit = 0x0303;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_5 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*R768 (0x0300) AIF1 _CONTROL */
	/*AIF1_CONTROL_1*/
	/*AIF1ADCL_SRC (0), AIF1ADCR_SRC (1), AIF1ADC_TDM (0), AIF1_BLCK_INV (0), AIF1_LRCLK_INV (0), AIF1_WL[1:0] (00) 16 BITS, AIF1_FMT [1:0] (10) I2S FORMAT */
	tx_bit = 0x4010;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CNTRL_1 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*R771 (0x0303) */
	/*AIF1 BCLK*/
	/*AIF1_BCLK_DIV (00000) BCLK1 RATE = AIF1CLK */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_BCLK , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*R772 (0x0304) */
	/*AIF1ADC_LRCLK_DIR (1) ADCLRCLK1 (ENABLED IN SLAVE MODE) */
	/* AIF1ADC_RATE [10:0] (0X0040) DEFAULT VALUE */

	tx_bit = 0x0840;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1ADC_LRCLK , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);



	/*R773 (0x0305) AIF1DACLRCLK */
	/*AIF1DAC_LRCLK_DIR (1) LRCLK1 ENABLED IN SLAVE MODE */
	/* AIF1DAC_RATE [10:0] */

	tx_bit = 0x0840;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1DAC_LRCLK , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);













	/*R769 (0x0301) AIF1_CONTROL_2 */
	/*AIF1DACL_SRC (0) LEFT ADC RECEIVES LEFT INTERFACE, AIF1DACR_SRC (1) RIGHT ADC RECEIVES RIGHT INTERFACE DATA */
	/*AIF1DAC_BOOST [1:0] */

	tx_bit = 0x2000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, AIF1_CNTRL_2 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*SIDETONE R1569 (0x0621) */
	/*ST_HPF_CUT [2:0] (110) 45 HZ, ST_HPF (1) ENABLED, STR_SEL (0), STL_SEL (0) */
	tx_bit = 0x0300;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SIDE_TONE , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*DAC1_MIXR_VOL R1536 (0x0600) */
	/* ADCR_DAC1_VOL (1111) SIDETONE GAIN (Db) (0), ADCR_DAC1_V (1111) 0 DB, ADCL_DAC1_V (1111) 0 DB */
	tx_bit = 0x01EF;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_MIX_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*DAC1_LEFT_MIXER_ROUTING R1537 (0x0601) */
	/*ADCR_TO_DAC1L (0) DISABLED, ADCL_DAC1l (1) ENABLED, AIF2DACL_TO_DAC1L (0) DISABLED, AIF1DAC2L_TO_DAC1L (0) DISABLED */
	/*AIF1DAC1L_TO_DAC1L (0) DISABLED */
	tx_bit = 0x0010;
	rx_bit = shift_register(tx_bit);

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1L_MIXR_ROUTING	 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	tx_bit = 0x0020;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1R_MIXR_ROUTING	 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*POWER MANAGEMENT 5 R5 (0x0005) */
	/* AIF2DACL_ENA (0), AIF2DACR_ENA (0), AIF1DAC2L_ENA (0), AIF1DAC2R_ENA (0), AIF1DAC1L_ENA (0) DISABLED, AIF1DAC1R_ENA (0) DISABLED */
	/*DAC2L_ENA (0) DISABLED, DAC2R_ENA (0) DISABLED */
	/*DAC2L_ENA (0) DISABLED LEFT DAC2 DISABLED, DAC2R_ENA (0) DISABLED RIGHT DAC2 DISABLED */
	/*DAC1L_ENA (1) LEFT DAC 1 ENABLED, DAC1R_ENA (1) RIGHT DAC1 ENABLED */
	/*VERIFIED 3-12-25*/

	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_5, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*DAC DIGITAL VOLUME SECTION */
	/* DAC1 LEFT VOLUME REGISTER R1552 (0X0610) */
	/*DAC1L_MUTE (0) UN-MUTE, DAC1_VU (1) DAC1L AND DAC1R VOLUME UPDATE , DAC1L_VOL (11000000) 0 DB */

	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_LEFT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/* DAC1 RIGHT VOLUME REGISTER R1553 (0X0611) */
	/*DAC1R_MUTE (0) UN-MUTE, DAC1_VU (1) DAC1L AND DAC1R VOLUME UPDATE , DAC1R_VOL (11000000) 0 DB */

	tx_bit = 0x01C0;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC1_RIGHT_VOL , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*DAC SOFT MUTE CONTROL */
	/*DAC SOFTMUTE REGISTER R1556 (0X0614) */
	/* DAC_SOFTMUTEMODE (0) DAC UNMUTE RAMP SELECT DISABLING SOFT MUTE, DAC_MUTERATE (0) DAC SOFT MUTE RAMP RATE */
	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, DAC_SOFTMUTE , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);


	/*POWER MANAGEMENT 1*/
	/*SPKOUTR_ENA (1), SPKOUTL_ENA (1), HPOUT2_ENA (0), HPOUT1L_ENA (0), HPOUT1R_ENA (0), MICB2_ENA (0), MICB1_ENA (0), VMID_SEL (01), BIAS_ENA (1)
	 */
	tx_bit = 0x3003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_1 , I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*PWR_MANAGE_3*/
	/*LINEOUT1N ENA (0), LINEOUT1P_ENA (0), LINEOUT2N ENA (0), LINEOUT2P_ENA (0), SPKRVOL ENA (1),
	* SPKLVOL_ENA (1), MIXOUTLVOL ENA (0), MIXOUTRVOL ENA (0), MIXOUTL ENA (0), MIXOUTR ENA (0),
	* REST OF THE 4 BITS ARE ZERO
	*/
	/*VERIFIED 3-3-25*/
	tx_bit = 0x0300;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, PWR_MANAGE_3, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*R54 SPEAKER MIXER (0X0034) */
	/*DAC2L_TO_SPKMIXL (0) MUTE, MIXINL_TO_SPKMIXL (0) MUTE, IN1LP_TO_SPKMIXL (0) MUTE, MKIXOUTL_TO_SPKMIXL (0) MUTE,
	 * DAC1L_TO_SPKMIXL (1) UNMITE
	 */

	tx_bit = 0x0003;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SPKR_MIXR, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*SPKMIXL_ATTENUATION R34 (0X0022) */
	/*SPKAB_REF_SEL (1), DAC1L_SPKMIXL_VOL (0) O DB, SPKMIXL_VOL (00) O DB */

	tx_bit = 0x0000;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SPEAKER_MIXR_ATN_L, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	tx_bit = 0x0100;
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, SPEAKER_MIXR_ATN_R, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	/*SPEAKER VOLUME LEFT AND SPEAKER VOLUME RIGHT*/
	/*REGISTER R38 (0X0026) AND REGISTER R39 (0X0027)*/
	/*SPKOUT_VU (1), SPKOUTR_ZC (0), SPKOUTR_MUTE_N (1), SPKOUTR_VOL (111100)*/
	//tx_bit = 0x017C;
	//tx_bit = 0x0173;
	tx_bit = 0x017F; //+6db output volume
	rx_bit = shift_register(tx_bit);
	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,  SPEAKER_VOL_L, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS,  SPEAKER_VOL_R, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

//	/* R37 CLASS D (0X0025)
//	* SPKOUTL_BOOST [2:0] (100) 2.00X BOOST (+6.0 DB)
//	* SPKOUTR_BOOST [2:0] (100) 2.00X BOOST (+6.0DB)
//	*/
//
//	tx_bit = 0x0164;
//	rx_bit = shift_register(tx_bit);
//	ret = HAL_I2C_Mem_Write(&hi2c4, WM8994_ADDRESS, CLASS_D, I2C_MEMADD_SIZE_16BIT, &rx_bit, 2, 1000);

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
