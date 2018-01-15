/**
  ******************************************************************************
  * @file    usbd_custom_hid_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Custom HID device class core defines
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_CUSTOMHID_CORE_H
#define __GD32F1X0_USBD_CUSTOMHID_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_enum.h"

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

/** @defgroup USB_CustomHID_Class_Exported_Defines
  * @{
  */
#define USB_CUSTOMHID_DESC_SIZE             9

#define CUSTOMHID_REPORT_DESC_SIZE          95
#define CUSTOMHID_CONFIG_DESC_SIZE          41

#define CUSTOMHID_DESC_TYPE                 0x21
#define CUSTOMHID_REPORT_DESCTYPE           0x22

#define GET_REPORT                          0x01
#define GET_IDLE                            0x02
#define GET_PROTOCOL                        0x03
#define SET_REPORT                          0x09
#define SET_IDLE                            0x0A
#define SET_PROTOCOL                        0x0B

/**
  * @}
  */

/** @defgroup USB_CustomHID_Exported_Variables
  * @{
  */

extern USBD_Class_cb_TypeDef  USBD_CUSTOMHID_cb;

/**
  * @}
  */

/** @defgroup USB_CustomHID_Exported_Functions
  * @{
  */

uint8_t  USBD_CUSTOMHID_SendReport (USB_DEVICE_HANDLE *pudev,
                                             uint8_t *report,
                                             uint16_t Len);

/**
  * @}
  */

#endif  /* __GD32F1X0_USB_CUSTOMHID_CORE_H */

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
