/**
  ******************************************************************************
  * @file    usbd_printer_cdc_wrapper.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   header file for the usbd_printer_cdc_wrapper.c file.
  ******************************************************************************
  */  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_PRINTER_CDC_WRAPPER_H
#define __GD32F1X0_USB_PRINTER_CDC_WRAPPER_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_printer_core.h"
#include "usbd_cdc_core.h"

/* Exported defines ----------------------------------------------------------*/
#define PRINTER_INTERFACE       0x00
#define CDC_COM_INTERFACE       0x01

/* Exported macros -----------------------------------------------------------*/
#define USB_PRINTER_CDC_CONFIG_DESC_SIZE (USB_PRINTER_CONFIG_DESC_SIZE - 9 + \
                                            USB_CDC_CONFIG_DESC_SIZE + 8)

/* Exported variables --------------------------------------------------------*/
extern USBD_Class_cb_TypeDef USBD_PRINTER_CDC_cb;

#endif  /* __GD32F1X0_USB_PRINTER_CDC_WRAPPER_H_ */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
