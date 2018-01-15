/**
  ******************************************************************************
  * @file    usbd_msc_bot.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB BBB(Bulk/Bulk/Bulk) protocol core functions
  * @note    BBB means Bulk-only transport protocol for usb mass storage device
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_bot.h"

/* Private variables ---------------------------------------------------------*/
uint16_t             MSC_BBB_DataLen;
uint8_t              MSC_BBB_State;
uint8_t              MSC_BBB_Status;
uint8_t              MSC_BBB_Data[MSC_MEDIA_PACKET_SIZE];

MSC_BBB_CBW_TypeDef  MSC_BBB_cbw;
MSC_BBB_CSW_TypeDef  MSC_BBB_csw;

/* Private function prototypes -----------------------------------------------*/
static void  MSC_BBB_CBW_Decode (USB_DEVICE_HANDLE *pudev);

static void  MSC_BBB_SendData   (USB_DEVICE_HANDLE *pudev,
                                            uint8_t *pbuf,
                                            uint16_t Len);

static void  MSC_BBB_Abort      (USB_DEVICE_HANDLE *pudev);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the BBB process
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
void  MSC_BBB_Init (USB_DEVICE_HANDLE *pudev)
{
    MSC_BBB_State = BBB_IDLE;
    MSC_BBB_Status = BBB_STATUS_NORMAL;

    USBD_STORAGE_fops->Init(0);

    /* Prepare endpoint to receive first BBB CBW */
    USB_EP_Rx (pudev,
               MSC_OUT_EP,
               (uint8_t *)&MSC_BBB_cbw,
               BBB_CBW_LENGTH);
}

/**
  * @brief  Reset the BBB machine
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
void  MSC_BBB_Reset (USB_DEVICE_HANDLE *pudev)
{
    MSC_BBB_State = BBB_IDLE;
    MSC_BBB_Status = BBB_STATUS_RECOVERY;

    USB_EP_Rx (pudev,
               MSC_OUT_EP,
               (uint8_t *)&MSC_BBB_cbw,
               BBB_CBW_LENGTH);
}

/**
  * @brief  De-Initialize the BBB Machine
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
void  MSC_BBB_DeInit (USB_DEVICE_HANDLE *pudev)
{
    MSC_BBB_State = BBB_IDLE;
}

/**
  * @brief  Handle BBB IN data stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval None
  */
void MSC_BBB_DataIn (USB_DEVICE_HANDLE *pudev, uint8_t EpID)
{
    switch (MSC_BBB_State)
    {
        case BBB_DATA_IN:
            SCSI_ProcessRead(MSC_BBB_cbw.bCBWLUN);
            break;

        case BBB_SEND_DATA:
        case BBB_LAST_DATA_IN:
            MSC_BBB_SendCSW (pudev, CSW_CMD_PASSED);
            break;

        default:
            break;
    }
}
/**
  * @brief  Handle BBB OUT data stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval None
  */
void  MSC_BBB_DataOut (USB_DEVICE_HANDLE *pudev, uint8_t EpID)
{
    switch (MSC_BBB_State)
    {
        case BBB_IDLE:
            MSC_BBB_CBW_Decode(pudev);
            break;

        case BBB_DATA_OUT:
            SCSI_ProcessWrite(MSC_BBB_cbw.bCBWLUN);
            break;

        default:
            break;
    }
}

/**
  * @brief  Decode the CBW command and set the BBB state machine accordingly
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  MSC_BBB_CBW_Decode (USB_DEVICE_HANDLE *pudev)
{
    uint16_t RxLen = pudev->dev.out_ep[MSC_OUT_EP].xfer_count;

    MSC_BBB_csw.dCSWTag = MSC_BBB_cbw.dCBWTag;
    MSC_BBB_csw.dCSWDataResidue = MSC_BBB_cbw.dCBWDataTransferLength;

    if ((RxLen != BBB_CBW_LENGTH) ||
        (MSC_BBB_cbw.dCBWSignature != BBB_CBW_SIGNATURE) ||
          (MSC_BBB_cbw.bCBWLUN > 1) ||
            (MSC_BBB_cbw.bCBWCBLength < 1) ||
              (MSC_BBB_cbw.bCBWCBLength > 16))
    {
        SCSI_SenseCode(MSC_BBB_cbw.bCBWLUN,
                       ILLEGAL_REQUEST,
                       INVALID_CDB);

        MSC_BBB_Status = BBB_STATUS_ERROR;
        MSC_BBB_Abort(pudev);
    }
    else
    {
        if(SCSI_ProcessCmd(pudev,
                           MSC_BBB_cbw.bCBWLUN,
                           &MSC_BBB_cbw.CBWCB[0]) < 0)
        {
            MSC_BBB_Abort(pudev);
        }
        else if ((MSC_BBB_State != BBB_DATA_IN) && 
                  (MSC_BBB_State != BBB_DATA_OUT) &&
                   (MSC_BBB_State != BBB_LAST_DATA_IN)) 
        {
            if(MSC_BBB_DataLen > 0)
            {
                MSC_BBB_SendData(pudev,
                                 MSC_BBB_Data, 
                                 MSC_BBB_DataLen);
            }
            else if (MSC_BBB_DataLen == 0)
            {
                MSC_BBB_SendCSW (pudev, CSW_CMD_PASSED);
            }
        }
    }
}

/**
  * @brief  Send the requested data
  * @param  pudev: pointer to usb device instance
  * @param  buf: pointer to data buffer
  * @param  Len: data Length
  * @retval None
  */
static void  MSC_BBB_SendData (USB_DEVICE_HANDLE *pudev,
                                           uint8_t* buf,
                                           uint16_t Len)
{
    Len = MIN(MSC_BBB_cbw.dCBWDataTransferLength, Len);

    MSC_BBB_csw.dCSWDataResidue -= Len;
    MSC_BBB_csw.bCSWStatus = CSW_CMD_PASSED;
    MSC_BBB_State = BBB_SEND_DATA;

    USB_EP_Tx(pudev, MSC_IN_EP, buf, Len);
}

/**
  * @brief  Send the CSW(command status wrapper)
  * @param  pudev: pointer to usb device instance
  * @param  status: CSW status
  * @retval None
  */
void  MSC_BBB_SendCSW (USB_DEVICE_HANDLE *pudev, uint8_t CSW_Status)
{
    MSC_BBB_csw.dCSWSignature = BBB_CSW_SIGNATURE;
    MSC_BBB_csw.bCSWStatus = CSW_Status;
    MSC_BBB_State = BBB_IDLE;

    USB_EP_Tx(pudev,
              MSC_IN_EP,
              (uint8_t *)&MSC_BBB_csw,
              BBB_CSW_LENGTH);

    USB_EP_Rx(pudev,
              MSC_OUT_EP,
              (uint8_t *)&MSC_BBB_cbw,
              BBB_CBW_LENGTH);
}

/**
  * @brief  Abort the current transfer
  * @param  pudev: pointer to usb device instance
  * @retval status
  */
static void  MSC_BBB_Abort (USB_DEVICE_HANDLE *pudev)
{
    if ((MSC_BBB_cbw.bmCBWFlags == 0) && 
         (MSC_BBB_cbw.dCBWDataTransferLength != 0) &&
          (MSC_BBB_Status == BBB_STATUS_NORMAL))
    {
        USB_EP_Stall(pudev, MSC_OUT_EP);
    }

    USB_EP_Stall(pudev, MSC_IN_EP);

    if(MSC_BBB_Status == BBB_STATUS_ERROR)
    {
        USB_EP_Rx (pudev,
                   MSC_OUT_EP,
                   (uint8_t *)&MSC_BBB_cbw,
                   BBB_CBW_LENGTH);
    }
}

/**
  * @brief  Complete the clear feature request
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval None
  */

void  MSC_BBB_CplClrFeature (USB_DEVICE_HANDLE *pudev, uint8_t EpID)
{
    if(MSC_BBB_Status == BBB_STATUS_ERROR)/* Bad CBW Signature */
    {
        USB_EP_Stall(pudev, MSC_IN_EP);
        MSC_BBB_Status = BBB_STATUS_NORMAL;
    }
    else if(((EpID & 0x80) == 0x80) && (MSC_BBB_Status != BBB_STATUS_RECOVERY))
    {
        MSC_BBB_SendCSW (pudev, CSW_CMD_FAILED);
    }
}

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
