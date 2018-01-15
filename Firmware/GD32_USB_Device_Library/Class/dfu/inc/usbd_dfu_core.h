/**
  ******************************************************************************
  * @file    usbd_dfu_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Header file for the usbd_dfu_core.c file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_DFU_CORE_H
#define __GD32F1X0_USBD_DFU_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_mal.h"
#include "usbd_enum.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_DFU_Class
  * @{
  */

/** @defgroup USB_DFU_Core_Exported_Defines
  * @{
  */
#define USB_DFU_CONFIG_DESC_SIZE      (18 + (9 * USBD_ITF_MAX_NUM))
#define USB_DFU_DESC_SIZE             9
#define DFU_DESC_TYPE                 0x21

/* Manifestation State */
#define Manifest_complete             0x00
#define Manifest_In_Progress          0x01

/* Special commands with download request */
#define GET_COMMANDS                  0x00
#define SET_ADDRESS_POINTER           0x21
#define ERASE                         0x41

/* Memory operation command */
#define CMD_ERASE                     0
#define CMD_WRITE                     1

/* Bit detach capable = bit 3 in bmAttributes field */
#define DFU_DETACH_MASK               (uint8_t)(0x10)

/**
  * @}
  */

/** @defgroup USB_DFU_Core_Exported_Types
  * @{
  */

/* DFU device state defines */
typedef enum _DFU_STATE
{
    STATE_appIDLE = 0x00,
    STATE_appDETACH,
    STATE_dfuIDLE,
    STATE_dfuDNLOAD_SYNC ,
    STATE_dfuDNBUSY,
    STATE_dfuDNLOAD_IDLE,
    STATE_dfuMANIFEST_SYNC,
    STATE_dfuMANIFEST,
    STATE_dfuMANIFEST_WAIT_RESET,
    STATE_dfuUPLOAD_IDLE,
    STATE_dfuERROR
}
DFU_STATE;

/* DFU device status defines */
typedef enum _DFU_STATUS
{
    STATUS_OK = 0x00,
    STATUS_errTARGET,
    STATUS_errFILE,
    STATUS_errWRITE,
    STATUS_errERASE,
    STATUS_errCHECK_ERASED,
    STATUS_errPROG,
    STATUS_errVERIFY,
    STATUS_errADDRESS,
    STATUS_errNOTDONE,
    STATUS_errFIRMWARE,
    STATUS_errVENDOR,
    STATUS_errUSBR,
    STATUS_errPOR,
    STATUS_errUNKNOWN,
    STATUS_errSTALLEDPKT
}
DFU_STATUS;

/* DFU class-specific requests */
typedef enum _DFU_REQUESTS
{
    DFU_DETACH = 0,
    DFU_DNLOAD,
    DFU_UPLOAD,
    DFU_GETSTATUS,
    DFU_CLRSTATUS,
    DFU_GETSTATE,
    DFU_ABORT
}
DFU_REQUESTS;

typedef  void  (*pAppFunction) (void);

/**
  * @}
  */

/** @defgroup USB_DFU_Core_Exported_Macros
  * @{
  */

/**********  Descriptor of DFU interface 0 Alternate setting n ****************/  
#define USBD_DFU_IF_DESC(n)   0x09,   /* bLength: interface descriptor size */ \
                              USB_DESCTYPE_INTERFACE,   /* bDescriptorType */ \
                              0x00,   /* bInterfaceNumber: number of interface */ \
                              (n),    /* bAlternateSetting: alternate setting */ \
                              0x00,   /* bNumEndpoints: no use non control endpoint */ \
                              0xFE,   /* bInterfaceClass: application specific class code */ \
                              0x01,   /* bInterfaceSubClass : device firmware upgrade code */ \
                              0x02,   /* nInterfaceProtocol: DFU mode protocol */ \
                              USBD_INTERFACE_STR_IDX + (n) + 1 /* iInterface: index of string descriptor */

/**
  * @}
  */

/** @defgroup USB_DFU_Core_Exported_Variables
  * @{
  */
extern USBD_Class_cb_TypeDef USBD_DFU_cb;

/**
  * @}
  */

#endif  /* __GD32F1X0_USBD_DFU_CORE_H */

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
