/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "xprintf.h"
#include "board.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

typedef enum
{
  USART_CONSOLE,
  USB_CONSOLE,
} console_type_e;

void printfSampleRate(void)
{
  switch (getActialFreqVal())
    {
    case _10kHz_VAL: mprintfVCP("sample rate 10kHz\r\n"); break;
    case _5kHz_VAL: mprintfVCP("sample rate 5kHz\r\n"); break;
    case _1kHz_VAL: mprintfVCP("sample rate 1kHz\r\n"); break;
    case _500Hz_VAL: mprintfVCP("sample rate 500Hz\r\n"); break;
    case _250Hz_VAL: mprintfVCP("sample rate 250Hz\r\n"); break;
    case _150Hz_VAL: mprintfVCP("sample rate 150Hz\r\n"); break;
    case _50Hz_VAL: mprintfVCP("sample rate 50Hz\r\n"); break;
    case _10Hz_VAL: mprintfVCP("sample rate 10Hz\r\n"); break;
    case _5Hz_VAL: mprintfVCP("sample rate 5Hz\r\n"); break;
    case _500mHz_VAL: mprintfVCP("sample rate 500mHz\r\n"); break;
    case _250mHz_VAL: mprintfVCP("sample rate 250mHz\r\n"); break;
    case tim3Freq_num: break;
    }
}
static void printfHelp(void)
{
  mprintfVCP("h: this help\r\n\r\n");
  mprintfVCP("s: acquisition start\r\n");
  mprintfVCP("t: acquisition stop\r\n");
  mprintfVCP("0: show sample rate\r\n");
  mprintfVCP("j: set sample rate = 10kHz\r\n");
  mprintfVCP("k: set sample rate = 5kHz\r\n");
  mprintfVCP("l: set sample rate = 1kHz\r\n");
  mprintfVCP("1: set sample rate = 500Hz\r\n");
  mprintfVCP("2: set sample rate = 250Hz\r\n");
  mprintfVCP("3: set sample rate = 150Hz\r\n");
  mprintfVCP("4: set sample rate = 50Hz\r\n");
  mprintfVCP("5: set sample rate = 10Hz\r\n");
  mprintfVCP("6: set sample rate = 5Hz\r\n");
  mprintfVCP("7: set sample rate = 500mHz\r\n");
  mprintfVCP("8: set sample rate = 250mHz\r\n");
  mprintfVCP("r: board restart\r\n");
  mprintfVCP("g: get uptime\r\n");
  mprintfVCP("b: read portB state\r\n");
  mprintfVCP("p: console char test\r\n");
  mprintfVCP("i: show info\r\n");
  mprintfVCP("e: error handler test (!!!)\r\n");
  mprintfVCP("a: stm32 asser test (!!!)\r\n");
  mprintfVCP("w: iwdg test (!!!)\r\n");
  mprintfVCP("m: user assert test (!!!)\r\n");
}

/*
fw changelog:
1.0 - init version
1.1 - tim3 added, vcp open, close callbacks added
1.2 - sampling/2
*/
static void printfInfoHeader(void)
{
  mprintfVCP("logger_01, fw. ver. 1.3, SN:LG800A1\n");
  printfSampleRate();
}

static uint32_t whileCnt = 0;
volatile uint8_t startFlag = 0;

uint8_t getStartFlagState(void)
{
  return startFlag;
}

void setStartFlagState(uint8_t nState)
{
  startFlag = nState;
}

volatile static uint8_t InputsStates = 0;
volatile static uint8_t InputsStatesNegation = 0;
volatile static uint8_t lastInputsStates = 0;

static uint8_t getPortAState(void)
{
  return (0x00FF & GPIOA->IDR);
}

static void userConsoleHandler(uint8_t recKey, console_type_e consTyp)
{
  switch (recKey)
    {
    case 'h':
    case 'H':
      printfHelp();
      break;
    case 's':
    case 'S':
      if (getStartFlagState() == 0)
        {
          setStartFlagState(1);
          //mprintfVCP("---- START ----\r\n");
        }
      break;
    case 't':
    case 'T':
      if (getStartFlagState() == 1)
        {
          setStartFlagState(0);
          whileCnt = 0;
          //mprintfVCP("---- STOP ----\r\n");
          setLedState(1);
        }
      break;
    case 'e':
    case 'E':
      Error_Handler();
      break;
    case '0':
      printfSampleRate();
      break;
    case 'j':
    case 'J':
      tim3Init(_10kHz_VAL);
      mprintfVCP("sample rate 10kHz\r\n");
      break;
    case 'k':
    case 'K':
      tim3Init(_5kHz_VAL);
      mprintfVCP("sample rate 5kHz\r\n");
      break;
    case 'l':
    case 'L':
      tim3Init(_1kHz_VAL);
      mprintfVCP("sample rate 1kHz\r\n");
      break;
    case '1':
      tim3Init(_500Hz_VAL);
      mprintfVCP("sample rate 500Hz\r\n");
      break;
    case '2':
      tim3Init(_250Hz_VAL);
      mprintfVCP("sample rate 250Hz\r\n");
      break;
    case '3':
      tim3Init(_150Hz_VAL);
      mprintfVCP("sample rate 150Hz\r\n");
      break;
    case '4':
      tim3Init(_50Hz_VAL);
      mprintfVCP("sample rate 50Hz\r\n");
      break;
    case '5':
      tim3Init(_10Hz_VAL);
      mprintfVCP("sample rate 10Hz\r\n");
      break;
    case '6':
      tim3Init(_5Hz_VAL);
      mprintfVCP("sample rate 5Hz\r\n");
      break;
    case '7':
      tim3Init(_500mHz_VAL);
      mprintfVCP("sample rate 500mHz\r\n");
      break;
    case '8':
      tim3Init(_250mHz_VAL);
      mprintfVCP("sample rate 250mHz\r\n");
      break;
    case 'r':
    case 'R':
      boardRestart();
      break;
    case 'g':
    case 'G':
      mprintfVCP("getUptime() = %ds\n", (uint32_t)(NS2S(getUptime())));
      break;
    case 'b':
    case 'B':
      mprintfVCP("portAState = 0x%x\n", getPortAState());
      break;
    case 'w':
    case 'W':
      mprintfVCP("iwdg test...\n");
      while (1);
      break;
    case 'a':
    case 'A':
      assert_failed((uint8_t*)"stm32AssertTest.c", 123);
      break;
    case 'm':
    case 'M':
      massert(0);
      break;
    case 'p':
    case 'P':
      mprintfVCP("12345!@#$dfg<>:^&*abcABC - console test!?\r\n");
      break;
    case 'i':
    case 'I':
      printfInfoHeader();
      break;
    case ':':
      mprintfVCP("---------------------------\n");
      mprintfVCP("getUptime() = %ds\n", (uint32_t)(NS2S(getUptime())));
      mprintfVCP("whileCnt = %d\n", whileCnt);
      vcpStats();
      mprintfVCP("---------------------------\n");
      break;
    default:
      mprintf("char = %c\n", recKey);
      break;
    }
}

uint32_t actualUpTime = 0;
uint32_t consoleTimeUpdateTimestamp = 0;
uint32_t oneSecondUpdateTimeStamp = 0;
#define BLINKLED_PRESCALLER 5000
#define CONSOLE_UPDATE_TIMESTAMP_VAL 30 // seconds
#define ONE_SECOND_UPDATE_TIMESTAMP_VAL 1 // second
#define _USER_APP_RX_DATA_SIZE 64
#define _USER_APP_TX_DATA_SIZE 64
char ReceivedData[_USER_APP_RX_DATA_SIZE];
uint8_t ReceivedDataFlag = 0;
uint8_t recDbgChar;
char vcpDataBuff[_USER_APP_TX_DATA_SIZE] = {0};

void readPortBStateAndSendViaVcp(void)
{
  InputsStates = getPortAState();
  InputsStatesNegation = 0x000000FF & ~InputsStates;
  xsprintf((char*)vcpDataBuff, "D:%02x%02x\r\n", InputsStates, InputsStatesNegation);
  printfVCP(vcpDataBuff);

  if (lastInputsStates != InputsStates )
    {
      mprintf(C_GREEN "NEW = 0x%02x | LAST = 0x%02x\n", InputsStates, lastInputsStates);
      lastInputsStates = InputsStates;
    }
}

#define NVIC_VectTab_FLASH           ((uint32_t)0x08000000)
static void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{
  SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x08002800);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();

  usbDisconnect();
  delay_ms(500);
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  timerStartIt();
  showBootInfo();
  mprintf ("\n\n\n\nHAL_RCC_GetSysClockFreq = %d MHz\n", HAL_RCC_GetSysClockFreq() / 1000000);
  setLedState(1);
  iwdgStart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      if (getStartFlagState())
        {
          if (whileCnt % BLINKLED_PRESCALLER == 0)
            {
              blinkLed();
            }
        }

      actualUpTime = NS2S(getUptime());

      if (isUsartDataAvailable())
        {
          recDbgChar = receiveConsoleChar();
          if (recDbgChar)
            {
              userConsoleHandler(recDbgChar, USART_CONSOLE);
            }
        }

      if ((actualUpTime - oneSecondUpdateTimeStamp) >= ONE_SECOND_UPDATE_TIMESTAMP_VAL)
        {
          oneSecondUpdateTimeStamp = actualUpTime;
          usbVCPConnectionHandler();
        }

      if ((actualUpTime - consoleTimeUpdateTimestamp) >= CONSOLE_UPDATE_TIMESTAMP_VAL)
        {
          consoleTimeUpdateTimestamp = actualUpTime;
          mprintf("[main.c] %8d s passed...\n", consoleTimeUpdateTimestamp);
        }

      if (ReceivedDataFlag)
        {
          userConsoleHandler(ReceivedData[0], USB_CONSOLE);
          ReceivedDataFlag = 0;
        }

      if (whileCnt < 0xFFFFFFFF)
        whileCnt++;
      else
        {
          whileCnt = 0;
          mprintf(C_RED ">>>>>>>>>>>>> whileCnt = 0\n"C_NORMAL);
        }


      iwdgFeed();
      /* USER CODE END 3 */

    }
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
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

  /**Initializes the CPU, AHB and APB busses clocks
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  mprintfVCP("------------------------\n");
  mprintfVCP("------Error_Handler-----\n");
  mprintfVCP("------------------------\n");
  while (1)
    {
    }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file % s on line % d\r\n", file, line) */

  mprintf("A\n");
  mprintf("S\n");
  mprintf("S\n");
  mprintf("E\n");
  mprintf("R\n");
  mprintf("T\n");
  mprintf("\n");
  mprintf("F\n");
  mprintf("A\n");
  mprintf("I\n");
  mprintf("L\n");
  mprintf("E\n");
  mprintf("D\n");

  mprintfVCP("stm32 assert %s: %d\n", (char*)file, line);

  int s = 1;
  while (s);

  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
