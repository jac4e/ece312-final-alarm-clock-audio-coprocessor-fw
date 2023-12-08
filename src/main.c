/**
 ******************************************************************************
 * @file    ECE312-final-project-audio-coprocessor-fe/src/main.c
 * @author  Jacques Fourie
 * @brief   The audio coprocessor firmware for the ECE312 final project.
 ******************************************************************************
 * @attention
 *
 * Based off the DAC_SignalsGeneration example from the STM32CubeL4 firmware
 * The examples are licensed under BSD 3-Clause License and have the following copyright notice:
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <math.h>

#include "main.h"
#include "tracks.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_ADDRESS 0x69
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef DacHandle;
static DAC_ChannelConfTypeDef sConfig;
uint8_t sine_wav_data[8000];     // 1 Second of sine wave data
uint8_t triangle_wav_data[8000]; // 1 Second of triangle wave data
__IO uint8_t ubSelectedWavesForm = 0;
__IO uint8_t ubKeyPressed = SET;

uint8_t getMasterInput = 0;
uint8_t getTransferDirection = 0;
__IO uint32_t Transfer_Direction = 0;
__IO uint32_t Xfer_Complete = 0;

/* Buffer used for transmission */
uint8_t aTxBuffer[4];

/* Buffer used for reception */
uint8_t aRxBuffer[4];

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* Private function prototypes -----------------------------------------------*/
static void DAC_Ch1_TriangleConfig(void);
static void DAC_Ch1_SineConfig(void);
static void DAC_Ch1_TrackConfig(track_t track);
static void TIM6_Config(void);
void SystemClock_Config(void);
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // Initialize i2c buffers
  aRxBuffer[0] = 0x00;
  aRxBuffer[1] = 0x00;
  aRxBuffer[2] = 0x00;
  aRxBuffer[3] = 0x00;
  aTxBuffer[0] = 0xAA;
  aTxBuffer[1] = 0xBB;
  aTxBuffer[2] = 0xCC;
  aTxBuffer[3] = 0xDD;

  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  /* Configure LED3 */
  BSP_LED_Init(LED3);

  /* Configure PA.12 (Arduino D2) as input with External interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable and set PA.12 (Arduino D2) EXTI Interrupt to the lowest priority */
  NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x03);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));

  /*##-1- Configure the DAC peripheral #######################################*/
  DacHandle.Instance = DACx;

  /*##-2- Configure the TIM peripheral #######################################*/
  TIM6_Config();

  // Populate the sine wave data
  for (int i = 0; i < 8000; i++)
  {
    // 0.5 second wave at 440 Hz, 0.5 second silence
    if (i < 4000)
    {
      sine_wav_data[i] = (uint8_t)(127.5 * sin(2 * 3.14159265359 * 440 * i / 8000) + 127.5);
    }
    else
    {
      sine_wav_data[i] = 127.5;
    }
  }

  // Populate the triangle wave data
  for (int i = 0; i < 8000; i++)
  {
    // 0.5 second wave at 440 Hz, 0.5 second silence
    if (i < 4000)
    {
      triangle_wav_data[i] = (uint8_t)((255 / 3.14159265359) * asin(sin(2 * 3.14159265359 * 440 * i / 8000)) + 127.5);
    }
    else
    {
      triangle_wav_data[i] = 127.5;
    }
  }

  /*##-3- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance = I2Cx;
  // 2MHz with 32MHz clock
  I2cHandle.Init.Timing = 32000000 / 2000000;
  I2cHandle.Init.OwnAddress1 = I2C_ADDRESS;
  I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2 = 0xFF;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle, I2C_ANALOGFILTER_ENABLE);

  /*##-4- Put I2C peripheral in listen mode process ###########################*/
  if (HAL_I2C_EnableListen_IT(&I2cHandle) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /* Infinite loop */
  while (1)
  {
    if (getMasterInput == 1)
    {
      getMasterInput = 0;
      if (getTransferDirection == 0)
      {

        if (HAL_I2C_Slave_Seq_Transmit_IT(&I2cHandle, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
        {
          /* Transfer error in transmission process */
          Error_Handler();
        }
        else
        {
          HAL_Delay(1 * TXBUFFERSIZE);
        }
      }
      else
      {

        if (HAL_I2C_Slave_Seq_Receive_IT(&I2cHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
        {
          /* Transfer error in reception process */
          Error_Handler();
        }
        else
        {
          HAL_Delay(1 * RXBUFFERSIZE);
        }
      }
    }

    if (Xfer_Complete == 1)
    {
      //  HAL_Delay(1);
      /*##- Put I2C peripheral in listen mode proces ###########################*/
      if (HAL_I2C_EnableListen_IT(&I2cHandle) != HAL_OK)
      {
        /* Transfer error in reception process */
        Error_Handler();
      }
      Xfer_Complete = 0;
    }
  }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follows :
 *            System Clock source            = PLL (MSI)
 *            SYSCLK(Hz)                     = 80000000
 *            HCLK(Hz)                       = 80000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            MSI Frequency(Hz)              = 4000000
 *            PLL_M                          = 1
 *            PLL_N                          = 40
 *            PLL_R                          = 2
 *            PLL_P                          = 7
 *            PLL_Q                          = 4
 *            Flash Latency(WS)              = 4
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
      ;
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* Error if LED3 is slowly blinking (1 sec. period) */
  while (1)
  {
    BSP_LED_Toggle(LED3);
    HAL_Delay(1000);
  }
}

static void DAC_Ch1_TrackConfig(track_t track)
{
  /*##-1- Initialize the DAC peripheral ######################################*/
  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-1- DAC channel1 Configuration #########################################*/
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL) != HAL_OK)
  {
    /* Channel configuration Error */
    Error_Handler();
  }

  /*##-2- Enable DAC selected channel and associated DMA #############################*/
  if (HAL_DAC_Start_DMA(&DacHandle, DACx_CHANNEL, (uint32_t *)(track.data), track.length, DAC_ALIGN_8B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }
}

static void DAC_Ch1_SineConfig()
{
  /*##-1- Initialize the DAC peripheral ######################################*/
  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-1- DAC channel1 Configuration #########################################*/
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL) != HAL_OK)
  {
    /* Channel configuration Error */
    Error_Handler();
  }

  /*##-2- Enable DAC selected channel and associated DMA #############################*/
  if (HAL_DAC_Start_DMA(&DacHandle, DACx_CHANNEL, (uint32_t *)(sine_wav_data), 8000, DAC_ALIGN_8B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }
}

static void DAC_Ch1_TriangleConfig(void)
{
  /*##-1- Initialize the DAC peripheral ######################################*/
  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-1- DAC channel1 Configuration #########################################*/
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL) != HAL_OK)
  {
    /* Channel configuration Error */
    Error_Handler();
  }

  /*##-2- Enable DAC selected channel and associated DMA #############################*/
  if (HAL_DAC_Start_DMA(&DacHandle, DACx_CHANNEL, (uint32_t *)(triangle_wav_data), 8000, DAC_ALIGN_8B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_12)
  {
    /* Change the wave */
    ubKeyPressed = 1;

    /* Change the selected waves forms */
    ubSelectedWavesForm++;
    if (ubSelectedWavesForm > 3)
    {
      ubSelectedWavesForm = 0;
    }
  }
}

/**
 * @brief  TIM6 Configuration
 * @note   TIM6 configuration is based on APB1 frequency
 * @note   TIM6 Update event occurs each TIM6CLK/256
 * @param  None
 * @retval None
 */
void TIM6_Config(void)
{
  static TIM_HandleTypeDef htim;
  TIM_MasterConfigTypeDef sMasterConfig;

  /*##-1- Configure the TIM peripheral #######################################*/
  /* Time base configuration */
  htim.Instance = TIM6;

  htim.Init.Period = 32000000 / 4000;
  htim.Init.Prescaler = 0;
  htim.Init.ClockDivision = 0;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim);

  /* TIM6 TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

  /*##-2- Enable TIM peripheral counter ######################################*/
  HAL_TIM_Base_Start(&htim);
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED3 off: Transfer in transmission process is correct */
  BSP_LED_Off(LED3);
  Xfer_Complete = 1;
  aTxBuffer[0]++;
  aTxBuffer[1]++;
  aTxBuffer[2]++;
  aTxBuffer[3]++;
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED3 on: Transfer in reception process is correct */
  BSP_LED_On(LED3);

  Xfer_Complete = 1;
  aRxBuffer[0] = 0x00;
  aRxBuffer[1] = 0x00;
  aRxBuffer[2] = 0x00;
  aRxBuffer[3] = 0x00;
}

/**
 * @brief  Slave Address Match callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
 * @param  AddrMatchCode: Address Match Code
 * @retval None
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  getMasterInput = 1;
	getTransferDirection = TransferDirection;
}

/**
 * @brief  Listen Complete callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

/**
 * @brief  I2C error callbacks.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
   * 1- When Slave don't acknowledge it's address, Master restarts communication.
   * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
   */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
}

static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
 * @}
 */

/**
 * @}
 */
