/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
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
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/* USER CODE BEGIN PRIVATE_TYPES */
#include "board.h"
#include "xprintf.h"
/* USER CODE END PRIVATE_TYPES */
/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64
/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch (cmd)
    {
    case CDC_SEND_ENCAPSULATED_COMMAND:

      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

      break;

    case CDC_SET_COMM_FEATURE:

      break;

    case CDC_GET_COMM_FEATURE:

      break;

    case CDC_CLEAR_COMM_FEATURE:

      break;

    /*******************************************************************************/
    /* Line Coding Structure                                                       */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description                          */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits                            */
    /*                                        0 - 1 Stop bit                       */
    /*                                        1 - 1.5 Stop bits                    */
    /*                                        2 - 2 Stop bits                      */
    /* 5      | bParityType |  1   | Number | Parity                               */
    /*                                        0 - None                             */
    /*                                        1 - Odd                              */
    /*                                        2 - Even                             */
    /*                                        3 - Mark                             */
    /*                                        4 - Space                            */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
    /*******************************************************************************/
    case CDC_SET_LINE_CODING:

      break;

    case CDC_GET_LINE_CODING:

      break;

    case CDC_SET_CONTROL_LINE_STATE:

      break;

    case CDC_SEND_BREAK:

      break;

    default:
      break;
    }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  extern char ReceivedData[APP_RX_DATA_SIZE];
  extern uint8_t ReceivedDataFlag;

  int i;
  for (i = 0; i < APP_RX_DATA_SIZE; i++)
    {
      ReceivedData[i] = 0;
    }

  strncpy(ReceivedData, Buf, (*Len));
  ReceivedDataFlag = 1;

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  if (1)
    {
      USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
      if (hcdc->TxState != 0)
        {
          return USBD_BUSY;
        }
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
      result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    }
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static uint32_t vcpSendFails = 0;
static uint32_t vcpSendUses = 0;
void printfVCP(char* Buf)
{
  uint16_t buflen = strlen(Buf);
  if (buflen > APP_TX_DATA_SIZE)
    {
      massert(0);
    }

  vcpSendUses++;
  if (CDC_Transmit_FS((uint8_t*)Buf, buflen) != USBD_OK)
    {
      vcpSendFails++;
    }
}
void vcpStats(void)
{
  mprintfVCP("[VCP] used: %8d retries: %8d | OK:%8d\n", vcpSendUses, vcpSendFails, vcpSendUses - vcpSendFails);
}

uint8_t getUSBConnectionState(void)
{
  return hUsbDeviceFS.dev_state;
}

uint32_t getUSBDataLen(void)
{
  return hUsbDeviceFS.ep0_data_len;
}


void printfUSBStruct(void)
{
  mprintf("hUsbDeviceFS.id = %d\n", hUsbDeviceFS.id);
  mprintf("hUsbDeviceFS.dev_config = %d\n", hUsbDeviceFS.dev_config);
  mprintf("hUsbDeviceFS.dev_default_config = %d\n", hUsbDeviceFS.dev_default_config);
  mprintf("hUsbDeviceFS.dev_config_status = %d\n", hUsbDeviceFS.dev_config_status);
  mprintf("hUsbDeviceFS.ep0_state = %d\n", hUsbDeviceFS.ep0_state);
  mprintf("hUsbDeviceFS.ep0_data_len = %d\n", hUsbDeviceFS.ep0_data_len);
  mprintf("hUsbDeviceFS.dev_state = %d\n", hUsbDeviceFS.dev_state);
  mprintf("hUsbDeviceFS.dev_old_state = %d\n", hUsbDeviceFS.dev_old_state);
  mprintf("hUsbDeviceFS.dev_address = %d\n", hUsbDeviceFS.dev_address);
  mprintf("hUsbDeviceFS.dev_connection_status = %d\n", hUsbDeviceFS.dev_connection_status);
  mprintf("hUsbDeviceFS.dev_test_mode = %d\n", hUsbDeviceFS.dev_test_mode);
  mprintf("hUsbDeviceFS.dev_remote_wakeup = %d\n", hUsbDeviceFS.dev_remote_wakeup);

  //USBD_SpeedTypeDef       dev_speed;
  //USBD_EndpointTypeDef    ep_in[15];
  //USBD_EndpointTypeDef    ep_out[15];
  //USBD_SetupReqTypedef    request;
  //USBD_DescriptorsTypeDef *pDesc;
  //USBD_ClassTypeDef       *pClass;
  //void                    *pClassData;
  //void                    *pUserData;
  //void                    *pData;
}

volatile uint8_t vcpTerminalStateOpened = 0;
uint8_t getvcpTerminalState(void)
{
  return vcpTerminalStateOpened;
}

static void terminalOpenedCallback(void)
{
  mprintf(C_MAGENTA "terminalOpenedCallback\n"C_NORMAL);
  setStartFlagState(0); // acquire stop
}

static void terminalClosedCallback(void)
{
  mprintf(C_MAGENTA "terminalClosedCallback\n"C_NORMAL);
  setStartFlagState(0); // acquire stop
}

void usbVCPConnectionHandler(void)
{
  volatile uint8_t currentUsbState = getUSBConnectionState();
  volatile uint32_t usbDataLen = getUSBDataLen();

  if (usbDataLen > 0 && usbDataLen < 255)
    vcpTerminalStateOpened = 1;
  else
    vcpTerminalStateOpened = 0;

  static uint8_t vcpTerminalLastState;
  if (vcpTerminalLastState != vcpTerminalStateOpened)
    {
      if (vcpTerminalStateOpened)
        terminalOpenedCallback();
      else
        terminalClosedCallback();
      vcpTerminalLastState = vcpTerminalStateOpened;
    }

  static uint8_t lastUsbState;
  if (lastUsbState != currentUsbState)
    {
      mprintf("current: ");
      switch (currentUsbState)
        {
        case USBD_STATE_DEFAULT:
          mprintf(C_GREEN "USBD_STATE_DEFAULT\n"C_NORMAL);

          break;
        case USBD_STATE_ADDRESSED: mprintf(C_GREEN "USBD_STATE_ADDRESSED\n"C_NORMAL); break;
        case USBD_STATE_CONFIGURED: mprintf(C_GREEN "USBD_STATE_CONFIGURED\n"C_NORMAL); break;
        case USBD_STATE_SUSPENDED: mprintf(C_GREEN "USBD_STATE_SUSPENDED\n"C_NORMAL); break;
        default: break;
        }

      mprintf("old    : ");
      switch (lastUsbState)
        {
        case USBD_STATE_DEFAULT: mprintf(C_GREEN "USBD_STATE_DEFAULT\n"C_NORMAL); break;
        case USBD_STATE_ADDRESSED: mprintf(C_GREEN "USBD_STATE_ADDRESSED\n"C_NORMAL); break;
        case USBD_STATE_CONFIGURED: mprintf(C_GREEN "USBD_STATE_CONFIGURED\n"C_NORMAL); break;
        case USBD_STATE_SUSPENDED: mprintf(C_GREEN "USBD_STATE_SUSPENDED\n"C_NORMAL); break;
        default: mprintf("\n"); break;
        }
      lastUsbState = currentUsbState;
    }
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

