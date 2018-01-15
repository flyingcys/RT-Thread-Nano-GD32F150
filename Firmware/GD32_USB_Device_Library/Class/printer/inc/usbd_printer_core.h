/**
  ******************************************************************************
  * @file    usbd_printer_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB printer (USB printing device) header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_PRINTER_CORE_H
#define __GD32F1X0_USBD_PRINTER_CORE_H

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

/** @addtogroup USB_PRINTING_Class
  * @{
  */

/** @defgroup USB_PRINTING_Class_Exported_Types
  * @{
  */
#define USB_PRINTER_CONFIG_DESC_SIZE    32
#define DEVICE_ID_LEN                   102

#define GET_DEVICE_ID                   0x00
#define GET_PORT_STATUS                 0x01
#define SOFT_RESET                      0x02

/**
  * @}
  */

/** @defgroup USB_PRINTING_Class_Exported_Variables
  * @{
  */
extern USBD_Class_cb_TypeDef  USBD_Printer_cb;

/**
  * @}
  */

#endif  /* __GD32F1X0_USB_PRINTER_CORE_H */

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
