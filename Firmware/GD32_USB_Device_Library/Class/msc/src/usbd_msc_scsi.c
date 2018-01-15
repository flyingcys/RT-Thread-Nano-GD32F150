/**
  ******************************************************************************
  * @file    usbd_msc_scsi.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   This file provides all the USBD SCSI layer functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_scsi.h"

/* Private variables ---------------------------------------------------------*/
SCSI_Sense_TypeDef SCSI_Sense[SENSE_LIST_DEEPTH];

uint8_t  SCSI_Sense_Head;
uint8_t  SCSI_Sense_Tail;

uint32_t SCSI_blk_size;
uint32_t SCSI_blk_nbr;

uint32_t SCSI_blk_addr;
uint32_t SCSI_blk_len;

USB_DEVICE_HANDLE *cdev;

/* Private function prototypes -----------------------------------------------*/
static int8_t  SCSI_TestUnitReady      (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_Inquiry            (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_ReadFormatCapacity (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_ReadCapacity10     (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_RequestSense       (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_StartStopUnit      (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_AllowMediumRemoval (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_ModeSense6         (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_ModeSense10        (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_Write10            (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_Read10             (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_Verify10           (uint8_t Lun, uint8_t *params);
static int8_t  SCSI_CheckAddressRange  (uint8_t Lun,
                                 uint32_t BlkOffset,
                                 uint16_t BlkNbr);
//static int8_t  SCSI_ProcessRead        (uint8_t Lun);
//static int8_t  SCSI_ProcessWrite       (uint8_t Lun);
static int8_t  SCSI_Format_Cmd         (uint8_t Lun);
static int8_t  SCCI_ReadToc_Cmd        (uint8_t Lun, uint8_t *params);

/* Private function ----------------------------------------------------------*/

/**
  * @brief  Process SCSI commands
  * @param  pudev: pointer to device instance
  * @param  Lun: logical unit number
  * @param  params: Command parameters
  * @retval status
  */
int8_t  SCSI_ProcessCmd (USB_DEVICE_HANDLE  *pudev,
                                       uint8_t Lun,
                                       uint8_t *params)
{
    cdev = pudev;

    switch (params[0])
    {
        case SCSI_TEST_UNIT_READY:
            return SCSI_TestUnitReady(Lun, params);

        case SCSI_REQUEST_SENSE:
            return SCSI_RequestSense(Lun, params);

        case SCSI_INQUIRY:
            return SCSI_Inquiry(Lun, params);

        case SCSI_START_STOP_UNIT:
            return SCSI_StartStopUnit(Lun, params);

        case SCSI_ALLOW_MEDIUM_REMOVAL:
            return SCSI_AllowMediumRemoval(Lun, params);

        case SCSI_MODE_SENSE6:
            return SCSI_ModeSense6(Lun, params);

        case SCSI_MODE_SENSE10:
            return SCSI_ModeSense10(Lun, params);

        case SCSI_READ_FORMAT_CAPACITIES:
            return SCSI_ReadFormatCapacity(Lun, params);

        case SCSI_READ_CAPACITY10:
            return SCSI_ReadCapacity10(Lun, params);

        case SCSI_READ10:
            return SCSI_Read10(Lun, params);

        case SCSI_WRITE10:
            return SCSI_Write10(Lun, params);

        case SCSI_VERIFY10:
            return SCSI_Verify10(Lun, params);

        case SCSI_FORMAT_UNIT:
            return SCSI_Format_Cmd(Lun);

        case SCSI_READ_TOC_CMD:
            return SCCI_ReadToc_Cmd(Lun, params);

        default:
            SCSI_SenseCode(Lun,
                           ILLEGAL_REQUEST, 
                           INVALID_CDB);
            return -1;
    }
}


/**
  * @brief  Process SCSI Test Unit Ready Command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t SCSI_TestUnitReady (uint8_t Lun, uint8_t *params)
{
    /* case 9 : Hi > D0 */
    if (MSC_BBB_cbw.dCBWDataTransferLength != 0)
    {
        SCSI_SenseCode(MSC_BBB_cbw.bCBWLUN,
                       ILLEGAL_REQUEST,
                       INVALID_CDB);
        return -1;
    }

    if(USBD_STORAGE_fops->IsReady(Lun) != 0)
    {
        SCSI_SenseCode(Lun,
                       NOT_READY, 
                       MEDIUM_NOT_PRESENT);
        return -1;
    }

    MSC_BBB_DataLen = 0;

    return 0;
}

/**
  * @brief  Process Inquiry command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_Inquiry (uint8_t Lun, uint8_t *params)
{
    uint8_t* pPage;
    uint16_t Len;

    if (params[1] & 0x01)
    {
        /* Evpd is set */

        pPage = (uint8_t *)MSC_Page00_Inquiry_Data;
        Len = INQUIRY_PAGE00_LENGTH;
    }
    else
    {
        /* Evpd is 0 */

        pPage = (uint8_t *)&USBD_STORAGE_fops->pInquiryData[Lun * USBD_STD_INQUIRY_LENGTH];
        Len = pPage[4] + 5;

        if (params[4] <= Len)
        {
            Len = params[4];
        }
    }

    MSC_BBB_DataLen = Len;

    while (Len) 
    {
        Len--;
        MSC_BBB_Data[Len] = pPage[Len];
    }

    return 0;
}

/**
  * @brief  Process Read_Toc command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCCI_ReadToc_Cmd (uint8_t Lun, uint8_t *params)
{
    uint8_t* pPage;
    uint16_t Len;

    pPage = (uint8_t *)&USBD_STORAGE_fops->pRead_toc_data[Lun * USBD_STD_READ_TOC_CMD_LENGTH];
    Len = pPage[1] + 2;

    MSC_BBB_DataLen = Len;

    while (Len) 
    {
        Len--;
        MSC_BBB_Data[Len] = pPage[Len];
    }

    return 0;
}

/**
  * @brief  Process Read Capacity 10 command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_ReadCapacity10 (uint8_t Lun, uint8_t *params)
{
    if(USBD_STORAGE_fops->GetCapacity(Lun, &SCSI_blk_nbr, &SCSI_blk_size) != 0)
    {
        SCSI_SenseCode(Lun,
                       NOT_READY, 
                       MEDIUM_NOT_PRESENT);

        return -1;
    }
    else
    {
        uint32_t BlkNum = SCSI_blk_nbr - 1;

        MSC_BBB_Data[0] = (uint8_t)(BlkNum >> 24);
        MSC_BBB_Data[1] = (uint8_t)(BlkNum >> 16);
        MSC_BBB_Data[2] = (uint8_t)(BlkNum >> 8);
        MSC_BBB_Data[3] = (uint8_t)(BlkNum);

        MSC_BBB_Data[4] = (uint8_t)(SCSI_blk_size >> 24);
        MSC_BBB_Data[5] = (uint8_t)(SCSI_blk_size >> 16);
        MSC_BBB_Data[6] = (uint8_t)(SCSI_blk_size >> 8);
        MSC_BBB_Data[7] = (uint8_t)(SCSI_blk_size);

        MSC_BBB_DataLen = 8;

        return 0;
    }
}
/**
  * @brief  Process Read Format Capacity command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_ReadFormatCapacity (uint8_t Lun, uint8_t *params)
{
    uint32_t BlkSize;
    uint32_t BlkNum;
    uint16_t i;

    for(i = 0; i < 12; i++)
    {
        MSC_BBB_Data[i] = 0;
    }

    if(USBD_STORAGE_fops->GetCapacity(Lun, &BlkNum, &BlkSize) != 0)
    {
        SCSI_SenseCode(Lun,
                       NOT_READY, 
                       MEDIUM_NOT_PRESENT);
        return (-1);
    }
    else
    {
        uint32_t BlkNbr = BlkNum - 1;

        MSC_BBB_Data[3] = 0x08;
        MSC_BBB_Data[4] = (uint8_t)(BlkNbr >> 24);
        MSC_BBB_Data[5] = (uint8_t)(BlkNbr >> 16);
        MSC_BBB_Data[6] = (uint8_t)(BlkNbr >> 8);
        MSC_BBB_Data[7] = (uint8_t)(BlkNbr);

        MSC_BBB_Data[8] =  0x02;
        MSC_BBB_Data[9] =  (uint8_t)(BlkSize >> 16);
        MSC_BBB_Data[10] = (uint8_t)(BlkSize >> 8);
        MSC_BBB_Data[11] = (uint8_t)(BlkSize);

        MSC_BBB_DataLen = 12;

        return 0;
    }
}

/**
  * @brief  Process Mode Sense6 command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_ModeSense6 (uint8_t Lun, uint8_t *params)
{
    uint16_t len = 8;

    MSC_BBB_DataLen = len;

    while (len)
    {
        len--;
        MSC_BBB_Data[len] = MSC_Mode_Sense6_data[len];
    }

    return 0;
}

/**
  * @brief  Process Mode Sense10 command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_ModeSense10 (uint8_t Lun, uint8_t *params)
{
    uint16_t Len = 8;

    MSC_BBB_DataLen = Len;

    while (Len) 
    {
        Len--;
        MSC_BBB_Data[Len] = MSC_Mode_Sense10_data[Len];
    }

    return 0;
}

/**
  * @brief  Process Request Sense command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_RequestSense (uint8_t Lun, uint8_t *params)
{
    uint8_t i;

    for(i = 0; i < REQUEST_SENSE_DATA_LEN; i++)
    {
        MSC_BBB_Data[i] = 0;
    }

    MSC_BBB_Data[0] = 0x70;
    MSC_BBB_Data[7] = REQUEST_SENSE_DATA_LEN - 6;

    if((SCSI_Sense_Head != SCSI_Sense_Tail))
    {
        MSC_BBB_Data[2]  = SCSI_Sense[SCSI_Sense_Head].SenseKey;
        MSC_BBB_Data[12] = SCSI_Sense[SCSI_Sense_Head].ASCQ;
        MSC_BBB_Data[13] = SCSI_Sense[SCSI_Sense_Head].ASC;

        SCSI_Sense_Head++;

        if (SCSI_Sense_Head == SENSE_LIST_DEEPTH)
        {
            SCSI_Sense_Head = 0;
        }
    }

    MSC_BBB_DataLen = MIN(REQUEST_SENSE_DATA_LEN, params[4]);

    return 0;
}

/**
  * @brief  Load the last error code in the error list
  * @param  Lun: logical unit number
  * @param  SKey: sense key
  * @param  ASC: additional aense key
  * @retval None
  */
void  SCSI_SenseCode (uint8_t Lun, uint8_t SKey, uint8_t ASC)
{
    SCSI_Sense[SCSI_Sense_Tail].SenseKey = SKey;
    SCSI_Sense[SCSI_Sense_Tail].ASC = ASC << 8;
    SCSI_Sense_Tail++;

    if (SCSI_Sense_Tail == SENSE_LIST_DEEPTH)
    {
        SCSI_Sense_Tail = 0;
    }
}

/**
  * @brief  Process Start Stop Unit command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_StartStopUnit(uint8_t Lun, uint8_t *params)
{
    MSC_BBB_DataLen = 0;

    return 0;
}

/**
  * @brief  Process Allow Medium Removal command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_AllowMediumRemoval(uint8_t Lun, uint8_t *params)
{
    MSC_BBB_DataLen = 0;

    return 0;
}

/**
  * @brief  Process Read10 command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_Read10(uint8_t Lun, uint8_t *params)
{
    if(MSC_BBB_State == BBB_IDLE)
    {
        /* Direction is from device to host */
        if ((MSC_BBB_cbw.bmCBWFlags & 0x80) != 0x80)
        {
            SCSI_SenseCode(MSC_BBB_cbw.bCBWLUN,
                               ILLEGAL_REQUEST,
                               INVALID_CDB);
            return -1;
        }

        if(USBD_STORAGE_fops->IsReady(Lun) != 0)
        {
            SCSI_SenseCode(Lun,
                           NOT_READY, 
                           MEDIUM_NOT_PRESENT);
            return -1;
        }

        SCSI_blk_addr = (params[2] << 24) | (params[3] << 16) | \
                          (params[4] << 8) | params[5];

        SCSI_blk_len = (params[7] << 8) | params[8];

        if(SCSI_CheckAddressRange(Lun, SCSI_blk_addr, SCSI_blk_len) < 0)
        {
            return -1; /* error */
        }

        MSC_BBB_State = BBB_DATA_IN;
        SCSI_blk_addr *= SCSI_blk_size;
        SCSI_blk_len  *= SCSI_blk_size;

        /* cases 4,5 : Hi <> Dn */
        if (MSC_BBB_cbw.dCBWDataTransferLength != SCSI_blk_len)
        {
            SCSI_SenseCode(MSC_BBB_cbw.bCBWLUN,
                           ILLEGAL_REQUEST,
                           INVALID_CDB);

            return -1;
        }
    }

    MSC_BBB_DataLen = MSC_MEDIA_PACKET_SIZE;

    return SCSI_ProcessRead(Lun);
}

/**
  * @brief  Process Write10 command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_Write10 (uint8_t Lun, uint8_t *params)
{
    if (MSC_BBB_State == BBB_IDLE) /* Idle */
    {
        /* case 8 : Hi <> Do */
        if ((MSC_BBB_cbw.bmCBWFlags & 0x80) == 0x80)
        {
            SCSI_SenseCode(MSC_BBB_cbw.bCBWLUN,
                           ILLEGAL_REQUEST,
                           INVALID_CDB);

            return -1;
        }

        /* Check whether Media is ready */
        if(USBD_STORAGE_fops->IsReady(Lun) !=0)
        {
            SCSI_SenseCode(Lun,
                           NOT_READY, 
                           MEDIUM_NOT_PRESENT);

            return -1;
        }

        /* Check If media is write-protected */
        if(USBD_STORAGE_fops->IsWriteProtected(Lun) !=0)
        {
            SCSI_SenseCode(Lun,
                           NOT_READY, 
                           WRITE_PROTECTED);

            return -1;
        }

        SCSI_blk_addr = (params[2] << 24) | (params[3] << 16) | \
                          (params[4] << 8) | params[5];

        SCSI_blk_len = (params[7] << 8) | params[8];

        /* check if LBA address is in the right range */
        if(SCSI_CheckAddressRange(Lun, SCSI_blk_addr, SCSI_blk_len) < 0)
        {
            return -1; /* error */      
        }

        SCSI_blk_addr *= SCSI_blk_size;
        SCSI_blk_len  *= SCSI_blk_size;

        /* cases 3,11,13 : Hn,Ho <> D0 */
        if (MSC_BBB_cbw.dCBWDataTransferLength != SCSI_blk_len)
        {
            SCSI_SenseCode(MSC_BBB_cbw.bCBWLUN, 
                           ILLEGAL_REQUEST, 
                           INVALID_CDB);

            return -1;
        }

        /* Prepare EP to receive first data packet */
        MSC_BBB_State = BBB_DATA_OUT;
        USB_EP_Rx (cdev,
                   MSC_OUT_EP,
                   MSC_BBB_Data, 
                   MIN (SCSI_blk_len, MSC_MEDIA_PACKET_SIZE));
    }
    else /* Write Process ongoing */
    {
        return SCSI_ProcessWrite(Lun);
    }

    return 0;
}


/**
  * @brief  Process Verify10 command
  * @param  Lun: logical unit number
  * @param  params: command parameters
  * @retval status
  */
static int8_t  SCSI_Verify10 (uint8_t Lun, uint8_t *params)
{
    if ((params[1] & 0x02) == 0x02)
    {
        SCSI_SenseCode(Lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);

        return -1; /* Error, Verify Mode Not supported*/
    }

    if(SCSI_CheckAddressRange(Lun, SCSI_blk_addr, SCSI_blk_len) < 0)
    {
        return -1; /* error */
    }

    MSC_BBB_DataLen = 0;

    return 0;
}

/**
  * @brief  Check address range
  * @param  Lun: logical unit number
  * @param  BlkOffset: first block address
  * @param  BlkNbr: number of block to be processed
  * @retval status
  */
static int8_t  SCSI_CheckAddressRange (uint8_t Lun, uint32_t BlkOffset, uint16_t BlkNbr)
{
    if ((BlkOffset + BlkNbr) > SCSI_blk_nbr)
    {
        SCSI_SenseCode(Lun, ILLEGAL_REQUEST, ADDRESS_OUT_OF_RANGE);

        return -1;
    }

    return 0;
}

/**
  * @brief  Handle Read Process
  * @param  Lun: logical unit number
  * @retval status
  */
/* static  */ int8_t  SCSI_ProcessRead (uint8_t Lun)
{
    uint32_t len;

    len = MIN(SCSI_blk_len, MSC_MEDIA_PACKET_SIZE); 

    if(USBD_STORAGE_fops->Read(Lun,
                               MSC_BBB_Data,
                               SCSI_blk_addr,
                               len / SCSI_blk_size) < 0)
    {
        SCSI_SenseCode(Lun, HARDWARE_ERROR, UNRECOVERED_READ_ERROR);

        return -1; 
    }

    USB_EP_Tx (cdev, 
               MSC_IN_EP,
               MSC_BBB_Data,
               len);

    SCSI_blk_addr += len;
    SCSI_blk_len -= len;

    /* case 6 : Hi = Di */
    MSC_BBB_csw.dCSWDataResidue -= len;

    if (SCSI_blk_len == 0)
    {
        MSC_BBB_State = BBB_LAST_DATA_IN;
    }

    return 0;
}

/**
  * @brief  Handle write process
  * @param  Lun: logical unit number
  * @retval status
  */
/* static */ int8_t  SCSI_ProcessWrite (uint8_t Lun)
{
    uint32_t len = MIN(SCSI_blk_len, MSC_MEDIA_PACKET_SIZE); 

    if(USBD_STORAGE_fops->Write(Lun,
                                MSC_BBB_Data,
                                SCSI_blk_addr,
                                len / SCSI_blk_size) < 0)
    {
        SCSI_SenseCode(Lun, HARDWARE_ERROR, WRITE_FAULT);

        return -1;
    }

    SCSI_blk_addr += len; 
    SCSI_blk_len  -= len; 

    /* case 12 : Ho = Do */
    MSC_BBB_csw.dCSWDataResidue -= len;

    if (SCSI_blk_len == 0)
    {
        MSC_BBB_SendCSW (cdev, CSW_CMD_PASSED);
    }
    else
    {
        /* Prepare EP to Receive next packet */
        USB_EP_Rx (cdev,
                   MSC_OUT_EP,
                   MSC_BBB_Data, 
                   MIN (SCSI_blk_len, MSC_MEDIA_PACKET_SIZE)); 
    }

    return 0;
}

/**
  * @brief  Process Format Unit command
  * @param  Lun: logical unit number
  * @retval status
  */
static int8_t  SCSI_Format_Cmd (uint8_t Lun)
{
    return 0;
}

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
