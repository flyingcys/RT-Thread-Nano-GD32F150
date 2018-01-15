/**
  ******************************************************************************
  * @file    usbd_msc_cdc_wrapper.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   header file for the usbd_hid_cdc_wrapper.c file.
  ******************************************************************************
  */  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_MSC_CDC_WRAPPER_H
#define __GD32F1X0_USB_MSC_CDC_WRAPPER_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_core.h"
#include "usbd_cdc_core.h"

/* Exported defines ----------------------------------------------------------*/
#define MSC_INTERFACE       0x00
#define CDC_COM_INTERFACE   0x01

/* Exported macros -----------------------------------------------------------*/
#define USB_MSC_CDC_CONFIG_DESC_SIZE (USB_MSC_CONFIG_DESC_SIZE - 9 + \
                                        USB_CDC_CONFIG_DESC_SIZE + 16)

/* Exported variables --------------------------------------------------------*/
extern USBD_Class_cb_TypeDef USBD_MSC_CDC_cb;

#endif  /* __GD32F1X0_USB_MSC_CDC_WRAPPER_H_ */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
