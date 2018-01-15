/**
  ******************************************************************************
  * @file    usbd_msc_hid_wrapper.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   header file for the usbd_msc_hid_wrapper.c file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_HID_MSC_WRAPPER_H
#define __GD32F1X0_USB_HID_MSC_WRAPPER_H

/* Includes ------------------------------------------------------------------*/
#include  "usbd_enum.h"

/* Exported defines ----------------------------------------------------------*/
#define HID_INTERFACE 0x00
#define MSC_INTERFACE 0x01

/* Exported macros -----------------------------------------------------------*/
#define USB_HID_MSC_CONFIG_DESC_SIZE (CUSTOMHID_CONFIG_DESC_SIZE - 9 + USB_MSC_CONFIG_DESC_SIZE)

/* Exported variables --------------------------------------------------------*/
extern USBD_Class_cb_TypeDef  USBD_HID_MSC_cb;

#endif  /* __GD32F1X0_USB_HID_MSC_WRAPPER_H_ */
  
/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
