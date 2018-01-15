/**
  ******************************************************************************
  * @file    usbd_msc_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Header file for the usbd_msc_core.c file
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_MSC_CORE_H
#define __GD32F1X0_USBD_MSC_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_bot.h"

/* Exported defines ----------------------------------------------------------*/
#define BBB_GET_MAX_LUN              0xFE
#define BBB_RESET                    0xFF
#define USB_MSC_CONFIG_DESC_SIZE     32

#define MSC_EPIN_SIZE                MSC_DATA_PACKET_SIZE
#define MSC_EPOUT_SIZE               MSC_DATA_PACKET_SIZE

/* Exported variables --------------------------------------------------------*/
extern USBD_Class_cb_TypeDef  USBD_MSC_cb; 

#endif  /* __GD32F1X0_USBD_MSC_CORE_H */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
