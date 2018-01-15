/**
  ******************************************************************************
  * @file    usbd_enum.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB enumeration function prototypes
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_ENUM_H
#define __GD32F1X0_USBD_ENUM_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"
#include "usbd_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */

/** @defgroup USBD_Enumeration_Exported_Defines
  * @{
  */
#define IS_NOT_EP0(ep_addr)  ((ep_addr != 0x00) && (ep_addr != 0x80))

/**
  * @}
  */

/** @defgroup USBD_Enumeration_Exported_Functions
  * @{
  */
uint8_t  USBD_StdReq    (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
uint8_t  USBD_DevClsReq (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
uint8_t  USBD_VdrDefReq (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);

void  USBD_ParseSetupRequest (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
void  USBD_EnumError         (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);

void  IntToUnicode           (uint32_t Value, uint8_t *pbuf, uint8_t Len);
void  USBD_GetUnicodeString  (uint8_t *desc, uint8_t *unicode, uint16_t *len);


/**
  * @}
  */

#endif /* __GD32F1X0_USB_ENUM_H */

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
