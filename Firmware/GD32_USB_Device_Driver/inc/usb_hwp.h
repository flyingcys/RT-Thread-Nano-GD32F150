/**
  ******************************************************************************
  * @file    usb_hwp.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Hardware platform configuration header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_HWP_H
#define __GD32F1X0_USB_HWP_H

/* Includes ------------------------------------------------------------------*/
#include "usb_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Driver
  * @{
  */

/** @defgroup USB_Hwp_Exported_Functions
  * @{
  */
void  USB_HWP_Init         (void);
void  USB_HWP_ClockConfig  (void);
void  USB_HWP_USBINTConfig (void);

void  USB_HWP_DelayInit    (const uint32_t SYSCLK);
void  USB_HWP_uDelay       (const uint32_t usec);
void  USB_HWP_mDelay       (const uint32_t msec);

void  USB_Cable_Config     (TypeState NewState);

/**
  * @}
  */

#endif /* __GD32F1X0_USB_HWP_H */

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

