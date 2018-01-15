/**
  ******************************************************************************
  * @file    usbd_msc_data.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Header file for the usbd_msc_data.c file
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __GD32F1X0_USBD_MSC_DATA_H
#define __GD32F1X0_USBD_MSC_DATA_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_conf.h"

/* Exported defines ----------------------------------------------------------*/
#define MODE_SENSE6_LENGTH                 8
#define MODE_SENSE10_LENGTH                8
#define INQUIRY_PAGE00_LENGTH              7
#define FORMAT_CAPACITIES_LENGTH           20

/* Exported variables --------------------------------------------------------*/
extern const uint8_t MSC_Page00_Inquiry_Data[];  
extern const uint8_t MSC_Mode_Sense6_data[];
extern const uint8_t MSC_Mode_Sense10_data[]; 

#endif /* __GD32F1X0_USBD_MSC_DATA_H */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
