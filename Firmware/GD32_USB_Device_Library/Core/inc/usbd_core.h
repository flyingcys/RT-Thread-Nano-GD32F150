/**
  ******************************************************************************
  * @file    usbd_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB device core function protoype
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_CORE_H
#define __GD32F1X0_USBD_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usb_core.h"
#include "usb_int.h"
#include "usb_hwp.h"
#include "usbd_enum.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */

/** @defgroup USBD_Core_Exported_Types
  * @{
  */

/**
  * @brief USB device operation status
  */
typedef enum
{
    USBD_OK   = 0,
    USBD_BUSY,
    USBD_FAIL,
}
USBD_Status;

/**
  * @}
  */

/** @defgroup USBD_Core_Exported_Functions
  * @{
  */
void USBD_Init(USB_DEVICE_HANDLE *pudev,
               USBD_Desc_cb_TypeDef *device_desc,
               USBD_Class_cb_TypeDef *class_cb,
               USBD_User_cb_TypeDef *user_cb);

void  USBD_Connect    (USB_DEVICE_HANDLE *pudev);
void  USBD_Disconnect (USB_DEVICE_HANDLE *pudev);

uint8_t  USBD_SetupStage   (USB_DEVICE_HANDLE *pudev);
uint8_t  USBD_DataOutStage (USB_DEVICE_HANDLE *pudev, uint8_t EpID);
uint8_t  USBD_DataInStage  (USB_DEVICE_HANDLE *pudev, uint8_t EpID);

USBD_Status  USBD_DeInit (USB_DEVICE_HANDLE *pudev);
USBD_Status  USBD_ClrCfg (USB_DEVICE_HANDLE *pudev, uint8_t ConfigIndex);
USBD_Status  USBD_SetCfg (USB_DEVICE_HANDLE *pudev, uint8_t ConfigIndex);

/**
  * @}
  */

#endif /* __GD32F1X0_USBD_CORE_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
