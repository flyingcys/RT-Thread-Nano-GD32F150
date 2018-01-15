/**
  ******************************************************************************
  * @file    usbd_msc_mem.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Header for the STORAGE DISK file
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_MEM_H
#define __GD32F1X0_USBD_MEM_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/* Exported defines ----------------------------------------------------------*/
#define USBD_STD_INQUIRY_LENGTH    36
#define USBD_STD_READ_TOC_CMD_LENGTH    20

/* Exported types ------------------------------------------------------------*/
typedef struct _USBD_STORAGE
{
    int8_t (*Init)             (uint8_t Lun);
    int8_t (*GetCapacity)      (uint8_t Lun, uint32_t *block_num, uint32_t *block_size);
    int8_t (*IsReady)          (uint8_t Lun);
    int8_t (*IsWriteProtected) (uint8_t Lun);
    int8_t (*Read)             (uint8_t Lun, uint8_t *buf, uint32_t BlkAddr, uint16_t BlkLen);
    int8_t (*Write)            (uint8_t Lun, uint8_t *buf, uint32_t BlkAddr, uint16_t BlkLen);
    int8_t (*GetMaxLun)        (void);
    int8_t *pInquiryData;
    int8_t *pRead_toc_data;
}USBD_STORAGE_cb_TypeDef;

/* Exported functions ------------------------------------------------------- */ 
extern USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops;

#endif /* __GD32F1X0_USBD_MEM_H */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
