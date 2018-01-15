/**
  ******************************************************************************
  * @file    usbd_iap_mal.h
  * @author  MCU SD
  * @version V1.0
  * @date    6-Sep-2014
  * @brief   USB DFU device media access layer header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_IAP_MAL_H
#define __GD32F1X0_USBD_IAP_MAL_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_conf.h"
#include "usbd_iap_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_IAP
  * @{
  */

/** @defgroup USB_IAP_MAL_Exported_Types
  * @{
  */
typedef struct _IAP_MAL_PROPERTY
{
    const uint8_t* pStrDesc;
    uint8_t  (*pMAL_Init)      (void);
    uint8_t  (*pMAL_DeInit)    (void);
    uint8_t  (*pMAL_Erase)     (uint32_t Addr, uint32_t PageCount);
    uint8_t  (*pMAL_Write)     (uint8_t Data[], uint32_t Addr, uint32_t Len);
    uint8_t* (*pMAL_Read)      (uint32_t Addr, uint32_t Len);
    uint8_t  (*pMAL_CheckAdd)  (uint32_t Addr);
    const uint32_t EraseTimeout;
    const uint32_t WriteTimeout;
}
IAP_MAL_Property_TypeDef;

/**
  * @}
  */

/** @defgroup USB_DFU_MAL_Defines
  * @{
  */
typedef enum
{
    MAL_OK = 0,
    MAL_FAIL
} MAL_Status;

/**
  * @}
  */

/** @defgroup USB_DFU_MAL_Exported_Macros
  * @{
  */
#define _1st_BYTE(x)              (uint8_t)((x) & 0xFF)               /*!< Addressing cycle 1st byte */
#define _2nd_BYTE(x)              (uint8_t)(((x) & 0xFF00) >> 8)      /*!< Addressing cycle 2nd byte */
#define _3rd_BYTE(x)              (uint8_t)(((x) & 0xFF0000) >> 16)   /*!< Addressing cycle 3rd byte */

#define SET_POLLING_TIMEOUT(x)    buffer[1] = _1st_BYTE(x);\
                                  buffer[2] = _2nd_BYTE(x);\
                                  buffer[3] = _3rd_BYTE(x);

/**
  * @}
  */

/** @defgroup USB_DFU_MAL_Exported_Functions
  * @{
  */
uint8_t  IAP_MAL_Init      (void);
uint8_t  IAP_MAL_DeInit    (void);
uint8_t  IAP_MAL_Erase     (uint32_t Addr, uint32_t PageCount);
uint8_t  IAP_MAL_Write     (uint8_t Data[], uint32_t Addr, uint32_t Len);
uint8_t* IAP_MAL_Read      (uint32_t Addr, uint32_t Len);
uint8_t  IAP_MAL_GetStatus (uint32_t Addr, uint8_t Cmd, uint8_t *buffer);

/**
  * @}
  */

/** @defgroup USB_DFU_MAL_Exported_Variables
  * @{
  */
extern uint8_t  MAL_Buffer[TRANSFER_SIZE];

/**
  * @}
  */

#endif /* __GD32F1X0_USBD_DFU_MAL_H */

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
