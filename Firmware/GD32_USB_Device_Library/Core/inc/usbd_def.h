/**
  ******************************************************************************
  * @file    usbd_def.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   General defines for the usb device library 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_DEF_H
#define __GD32F1X0_USBD_DEF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_conf.h"
#include "stdbool.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */
  
/** @defgroup USBD_Protocol_Exported_Defines
  * @{
  */
#ifndef NULL
 #define NULL                                           0
#endif

#define  USB_DEV_QUALIFIER_DESC_LEN                     0x0A
#define  USB_CFG_DESC_LEN                               0x09

#define  USBD_LANGID_STR_IDX                            0x00
#define  USBD_MFC_STR_IDX                               0x01
#define  USBD_PRODUCT_STR_IDX                           0x02
#define  USBD_SERIAL_STR_IDX                            0x03
#define  USBD_CONFIG_STR_IDX                            0x04
#define  USBD_INTERFACE_STR_IDX                         0x05

#define  USB_STANDARD_REQ                               0x00
#define  USB_CLASS_REQ                                  0x20
#define  USB_VENDOR_REQ                                 0x40
#define  USB_REQ_MASK                                   0x60

#define  USB_REQTYPE_DEVICE                             0x00
#define  USB_REQTYPE_INTERFACE                          0x01
#define  USB_REQTYPE_ENDPOINT                           0x02
#define  USB_REQ_RECIPIENT_MASK                         0x1f

#define  USBREQ_GET_STATUS                              0x00
#define  USBREQ_CLEAR_FEATURE                           0x01
#define  USBREQ_SET_FEATURE                             0x03
#define  USBREQ_SET_ADDRESS                             0x05
#define  USBREQ_GET_DESCRIPTOR                          0x06
#define  USBREQ_GET_CONFIGURATION                       0x08
#define  USBREQ_SET_CONFIGURATION                       0x09
#define  USBREQ_GET_INTERFACE                           0x0A
#define  USBREQ_SET_INTERFACE                           0x0B

#define  USB_DESCTYPE_DEVICE                            0x01
#define  USB_DESCTYPE_CONFIGURATION                     0x02
#define  USB_DESCTYPE_STRING                            0x03
#define  USB_DESCTYPE_INTERFACE                         0x04
#define  USB_DESCTYPE_ENDPOINT                          0x05
#define  USB_DESCTYPE_DEVICE_QUALIFIER                  0x06
#define  USB_DESCTYPE_OTHER_SPEED_CONFIGURATION         0x07
#define  USB_DESCTYPE_BOS                               0x0F

#define  USB_STATUS_REMOTE_WAKEUP                       2
#define  USB_STATUS_SELF_POWERED                        1

#define  USB_FEATURE_ENDP_HALT                          0
#define  USB_FEATURE_REMOTE_WAKEUP                      1
#define  USB_FEATURE_TEST_MODE                          2

/**
  * @}
  */

/** @addtogroup USBD_Exported_Macros
  * @{
  */
#define SWAPBYTE(addr)       (((uint16_t)(*((uint8_t *)(addr)))) + \
                             (((uint16_t)(*(((uint8_t *)(addr)) + 1))) << 8))

#define LOWBYTE(x)           ((uint8_t)(x & 0x00FF))
#define HIGHBYTE(x)          ((uint8_t)((x & 0xFF00) >> 8))

#define MIN(a, b)            (((a) < (b)) ? (a) : (b))

/**
  * @}
  */

#endif /* __GD32F1X0_USBD_DEF_H */

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
