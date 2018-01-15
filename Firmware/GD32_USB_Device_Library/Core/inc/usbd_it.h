/**
  ******************************************************************************
  * @file    usbd_it.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB device interrupt events handle function defines
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_IT_H
#define __GD32F1X0_USBD_IT_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_pwr.h"
#include "usbd_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */

/** @defgroup USBD_Interrupt_Exported_Types
  * @{
  */

/* USB interrupt handle set */
typedef struct _USB_INTHandler
{
    uint8_t (* HP_ST)   (USB_CORE_HANDLE *pudev);
    uint8_t (* LP_ST)   (USB_CORE_HANDLE *pudev);
    uint8_t (* PMOU)    (USB_CORE_HANDLE *pudev);
    uint8_t (* Error)   (USB_CORE_HANDLE *pudev);
    uint8_t (* Reset)   (USB_CORE_HANDLE *pudev);
    uint8_t (* SOF)     (USB_CORE_HANDLE *pudev);
    uint8_t (* ESOF)    (USB_CORE_HANDLE *pudev);
    uint8_t (* Suspend) (USB_CORE_HANDLE *pudev);
    uint8_t (* WakeUp)  (USB_CORE_HANDLE *pudev);
}
USB_INTHandler_TypeDef;

/**
  * @}
  */

#endif /* __GD32F1X0_USBD_IT_H */

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
