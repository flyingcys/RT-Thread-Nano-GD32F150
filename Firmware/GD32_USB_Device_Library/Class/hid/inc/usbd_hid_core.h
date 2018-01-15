/**
  ******************************************************************************
  * @file    usbd_hid_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB mouse (USB HID device) header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_HID_CORE_H
#define __GD32F1X0_USBD_HID_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_HID_Class
  * @{
  */

/** @defgroup USB_HID_Class_Exported_Types
  * @{
  */
#define USB_HID_CONFIG_DESC_SIZE    34
#define USB_HID_DESC_SIZE           9
#define USB_HID_REPORT_DESC_SIZE    21

#define HID_DESC_TYPE               0x21
#define HID_REPORT_DESCTYPE         0x22

#define GET_REPORT                  0x01
#define GET_IDLE                    0x02
#define GET_PROTOCOL                0x03
#define SET_REPORT                  0x09
#define SET_IDLE                    0x0A
#define SET_PROTOCOL                0x0B

/**
  * @}
  */

/** @defgroup USB_HID_Class_Exported_Variables
  * @{
  */
extern USBD_Class_cb_TypeDef  USBD_HID_cb;

/**
  * @}
  */

/** @defgroup USB_HID_Class_Exported_Functions
  * @{
  */
uint8_t  USBD_HID_SendReport (USB_DEVICE_HANDLE *pudev,
                                       uint8_t *report,
                                       uint16_t Len);

/**
  * @}
  */

#endif  /* __GD32F1X0_USB_HID_CORE_H */

/**
  * @}
  */

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
