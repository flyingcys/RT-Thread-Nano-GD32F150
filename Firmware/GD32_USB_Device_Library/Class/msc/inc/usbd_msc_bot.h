/**
  ******************************************************************************
  * @file    usbd_msc_bot.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Header file for the usbd_msc_bot.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USBD_MSC_BOT_H
#define __GD32F1X0_USBD_MSC_BOT_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_msc_scsi.h"
#include "usbd_msc_mem.h"

/* Exported defines ----------------------------------------------------------*/
#define BBB_IDLE                       0       /* Idle state */
#define BBB_DATA_OUT                   1       /* Data Out state */
#define BBB_DATA_IN                    2       /* Data In state */
#define BBB_LAST_DATA_IN               3       /* Last Data In state */
#define BBB_SEND_DATA                  4       /* Send Immediate data */

#define BBB_CBW_SIGNATURE              0x43425355
#define BBB_CSW_SIGNATURE              0x53425355
#define BBB_CBW_LENGTH                 31
#define BBB_CSW_LENGTH                 13

/* CSW Status Definitions */
#define CSW_CMD_PASSED                 0x00
#define CSW_CMD_FAILED                 0x01
#define CSW_PHASE_ERROR                0x02

/* BOT Status */
#define BBB_STATUS_NORMAL              0
#define BBB_STATUS_RECOVERY            1
#define BBB_STATUS_ERROR               2

/* Exported types ------------------------------------------------------------*/
typedef struct _MSC_BBB_CBW
{
    uint32_t dCBWSignature;
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t  bmCBWFlags;
    uint8_t  bCBWLUN;
    uint8_t  bCBWCBLength;
    uint8_t  CBWCB[16];
}
MSC_BBB_CBW_TypeDef;


typedef struct _MSC_BBB_CSW
{
    uint32_t dCSWSignature;
    uint32_t dCSWTag;
    uint32_t dCSWDataResidue;
    uint8_t  bCSWStatus;
}
MSC_BBB_CSW_TypeDef;

/* Exported variables --------------------------------------------------------*/
extern uint8_t              MSC_BBB_Data[];
extern uint16_t             MSC_BBB_DataLen;
extern uint8_t              MSC_BBB_State;

extern MSC_BBB_CBW_TypeDef  MSC_BBB_cbw;
extern MSC_BBB_CSW_TypeDef  MSC_BBB_csw;

/* Exported functions ------------------------------------------------------- */ 
void  MSC_BBB_Init          (USB_DEVICE_HANDLE *pudev);
void  MSC_BBB_Reset         (USB_DEVICE_HANDLE *pudev);
void  MSC_BBB_DeInit        (USB_DEVICE_HANDLE *pudev);
void  MSC_BBB_DataIn        (USB_DEVICE_HANDLE *pudev, uint8_t EpID);
void  MSC_BBB_DataOut       (USB_DEVICE_HANDLE *pudev, uint8_t EpID);
void  MSC_BBB_SendCSW       (USB_DEVICE_HANDLE *pudev, uint8_t CSW_Status);
void  MSC_BBB_CplClrFeature (USB_DEVICE_HANDLE *pudev, uint8_t EpID);

#endif /* __GD32F1X0_USBD_MSC_BOT_H */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

