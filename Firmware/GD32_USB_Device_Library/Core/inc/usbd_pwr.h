/**
  ******************************************************************************
  * @file    usbd_pwr.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Power management header file
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_PWR_H
#define __GD32F1X0_USBD_PWR_H

/* Includes ------------------------------------------------------------------*/
#include "usb_hwp.h"
#include "usb_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */
  
/** @defgroup USBD_Power_Exported_Types
  * @{
  */
typedef enum _RESUME_STATE
{
    RESUME_EXTERNAL,
    RESUME_INTERNAL,
#ifdef LPM_ENABLED
    L1_RESUME_INTERNAL,
#endif
    RESUME_LATER,
    RESUME_ON,
    RESUME_OFF,
    RESUME_ESOF
}
RESUME_STATE;

/**
  * @}
  */

/** @defgroup USBD_Power_Exported_Variables
  * @{
  */
extern USB_CORE_HANDLE USB_Device_dev;

/**
  * @}
  */

/** @defgroup USBD_Power_Exported_Functions
  * @{
  */
void  USBD_Suspend (void);
void  USBD_Resume  (RESUME_STATE ResumeValue);

/**
  * @}
  */

#endif /* __GD32F1X0_USBD_PWR_H */

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
