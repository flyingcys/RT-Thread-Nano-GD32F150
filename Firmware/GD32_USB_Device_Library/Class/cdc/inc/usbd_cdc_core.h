/**
  ******************************************************************************
  * @file    usbd_cdc_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Header file for the usbd_cdc_core.c file.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_CDC_CORE_H
#define __GD32F1X0_USBD_CDC_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_CDC_Class
  * @{
  */

/** @defgroup USB_CDC_Class_Exported_Defines
  * @{
  */
#define USB_CDC_CONFIG_DESC_SIZE                (67)
#define USB_CDC_DESC_SIZE                       (58)

#define CDC_DESC_TYPE                           0x21

#define CDC_DATA_IN_PACKET_SIZE                 USB_CDC_DATA_PACKET_SIZE
#define CDC_DATA_OUT_PACKET_SIZE                USB_CDC_DATA_PACKET_SIZE

/* CDC Requests */
#define SEND_ENCAPSULATED_COMMAND               0x00
#define GET_ENCAPSULATED_RESPONSE               0x01
#define SET_COMM_FEATURE                        0x02
#define GET_COMM_FEATURE                        0x03
#define CLEAR_COMM_FEATURE                      0x04
#define SET_LINE_CODING                         0x20
#define GET_LINE_CODING                         0x21
#define SET_CONTROL_LINE_STATE                  0x22
#define SEND_BREAK                              0x23
#define NO_CMD                                  0xFF

/**
  * @}
  */

/** @defgroup USB_CDC_Class_Exported_Types
  * @{
  */
typedef struct _CDC_IF_FOP
{
    uint16_t (*pIf_Init)     (uint8_t  Config);
    uint16_t (*pIf_DeInit)   (void);
    uint16_t (*pIf_Ctrl)     (uint8_t Cmd, uint8_t* CmdBuf, uint16_t Len);
    uint16_t (*pIf_DataTx)   (uint8_t* DataBuf, uint16_t Len);
    uint16_t (*pIf_DataRx)   (uint8_t* DataBuf, uint16_t Len);
}
CDC_IF_Fop_TypeDef;

/**
  * @}
  */

/** @defgroup USB_CDC_Exported_Variables
  * @{
  */
extern USBD_Class_cb_TypeDef  USBD_CDC_cb;

/**
  * @}
  */

#endif  /* __GD32F1X0_USBD_CDC_CORE_H */

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
