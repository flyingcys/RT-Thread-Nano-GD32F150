/**
  ******************************************************************************
  * @file    usbd_dfu_core.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB DFU class core functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_core.h"

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

/* DFU Device library callbacks */
static uint8_t  USBD_DFU_Init               (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_DFU_DeInit             (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_DFU_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_DFU_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_DFU_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_DFU_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_DFU_EP0_TxSent         (void *pudev);
static uint8_t  USBD_DFU_EP0_RxReady        (void *pudev);
static uint8_t* USBD_DFU_GetCfgDesc         (uint8_t USBSpeed, uint16_t *len);
static uint8_t* USBD_DFU_GetUsrStringDesc   (uint8_t USBSpeed, uint8_t index, uint16_t *len);

/* DFU Requests management functions */
static void  DFU_Req_DETACH    (void *pudev, USB_DEVICE_REQ *req);
static void  DFU_Req_DNLOAD    (void *pudev, USB_DEVICE_REQ *req);
static void  DFU_Req_UPLOAD    (void *pudev, USB_DEVICE_REQ *req);
static void  DFU_Req_GETSTATUS (void *pudev);
static void  DFU_Req_CLRSTATUS (void *pudev);
static void  DFU_Req_GETSTATE  (void *pudev);
static void  DFU_Req_ABORT     (void *pudev);
static void  DFU_LeaveDFUMode  (void *pudev);

/** @defgroup USB_DFU_Core_Private_Variables
  * @{
  */
extern const uint8_t* USBD_DFU_StringDesc[];
extern uint8_t MAL_Buffer[];

/* State machine variables */
uint8_t DeviceStatus[6] =
{
    STATUS_OK,     /* bStatus: device status is OK  */
    0x00,          /* bwPollTimeout: 0ms */
    0x00,
    0x00,
    STATE_dfuIDLE, /* bState: device state is dfuIDLE */
    0x00           /* iString: index is 0 */
};

uint8_t Manifest_State = Manifest_complete;

/* Data Management variables */
static uint16_t BlockNum = 0;
static uint16_t Length = 0;
static uint32_t BaseAddress = APP_LOADED_ADDR;  /*!< User application base address to erase, program or read */
static __IO uint32_t USBD_DFU_AltSet = 0;

/* DFU interface class callbacks structure */
USBD_Class_cb_TypeDef  USBD_DFU_cb = 
{
    USBD_DFU_Init,
    USBD_DFU_DeInit,
    USBD_DFU_GetClassDescriptor,
    USBD_DFU_ClassReqHandle,
    USBD_DFU_GetInterface,
    USBD_DFU_SetInterface,
    USBD_DFU_EP0_TxSent,
    USBD_DFU_EP0_RxReady,
    NULL, /* DataIn */
    NULL, /* DataOut */
    NULL, /* SOF */
    USBD_DFU_GetCfgDesc,
    USBD_DFU_GetUsrStringDesc,
};

/* USB DFU device Configuration Descriptor */
const uint8_t USBD_DFU_CfgDesc[USB_DFU_CONFIG_DESC_SIZE] =
{
    0x09,         /* bLength: configuration descriptor size */
    USB_DESCTYPE_CONFIGURATION, /* bDescriptorType: configuration descriptor type */
    USB_DFU_CONFIG_DESC_SIZE,   /* wTotalLength: configuration descriptor set total length */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: configuration value */
    0x02,         /* iConfiguration: index of string descriptor describing the configuration */
    0x80,         /* bmAttributes: bus powered and not supports remote wakeup */
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */

    /**********  Descriptor of DFU interface 0 Alternate setting 0 **************/  
    USBD_DFU_IF_DESC(0), /* This interface is mandatory for all devices */

#if (USBD_ITF_MAX_NUM > 1)
    /**********  Descriptor of DFU interface 0 Alternate setting 1 **************/ 
    USBD_DFU_IF_DESC(1),
#endif

#if (USBD_ITF_MAX_NUM > 2)
    /**********  Descriptor of DFU interface 0 Alternate setting 2 **************/ 
    USBD_DFU_IF_DESC(2),
#endif

#if (USBD_ITF_MAX_NUM > 3)
    /**********  Descriptor of DFU interface 0 Alternate setting 3 **************/ 
    USBD_DFU_IF_DESC(3),
#endif

#if (USBD_ITF_MAX_NUM > 4)
    /**********  Descriptor of DFU interface 0 Alternate setting 4 **************/ 
    USBD_DFU_IF_DESC(4),
#endif

#if (USBD_ITF_MAX_NUM > 5)
    /**********  Descriptor of DFU interface 0 Alternate setting 5 **************/ 
    USBD_DFU_IF_DESC(5),
#endif

#if (USBD_ITF_MAX_NUM > 6)
    #error "ERROR: usbd_dfu_core.c: Modify the file to support more descriptors!"
#endif

    /******************** DFU functional descriptor********************/
    0x09,   /* blength: 9 bytes */
    DFU_DESC_TYPE,   /* bDescriptorType: DFU functional descriptor type */
    0x0B,   /* bmAttributes:
               bitCanDnload             = 1      (bit 0)
               bitCanUpload             = 1      (bit 1)
               bitManifestationTolerant = 0      (bit 2)
               bitWillDetach            = 1      (bit 3)
               Reserved                          (bit4-6)
               bitAcceleratedST         = 0      (bit 7) */
    0xFF,   /* wDetachTimeOut: 255 ms */
    0x00,
    LOWBYTE(TRANSFER_SIZE),       /* wTransferSize: 1024 bytes */
    HIGHBYTE(TRANSFER_SIZE),
    0x1A,                         /* bcdDFUVersion: DFU specifigcation release(DFU1.1A) */
    0x01
    /***********************************************************/
};

/**
  * @}
  */

/** @defgroup USB_DFU_Core_Private_Function
  * @{
  */

/**
  * @brief  Initializes the DFU interface.
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_Init (void *pudev, uint8_t ConfigIndex)
{
    /* Initialize the MAL(Media Access Layer) */
    DFU_MAL_Init();

    return USBD_OK;
}

/**
  * @brief  De-Initializes the DFU layer.
  * @param  pudev: pointer to device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_DeInit (void *pudev, uint8_t ConfigIndex)
{
    /* Restore device default state */
    DeviceStatus[0] = STATUS_OK;
    DeviceStatus[4] = STATE_dfuIDLE;

    BlockNum = 0;
    Length = 0;

    /* Deinitilialize the MAL(Media Access Layer) */
    DFU_MAL_DeInit();

    return USBD_OK;
}

/**
  * @brief  Get DFU class descriptor
  * @param  pudev: pointer to usb device instance
  * @param  req: device request
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    uint16_t len = 0;
    uint8_t  *pbuf = NULL;

    if((req->wValue >> 8) == DFU_DESC_TYPE)
    {
        pbuf = (uint8_t*)USBD_DFU_CfgDesc + 9 + (9 * USBD_ITF_MAX_NUM);
        len = MIN(USB_DFU_DESC_SIZE, req->wLength);
    }

    USB_CtlTx (pudev, pbuf, len);

    return USBD_OK;
}

/**
  * @brief  Handle the DFU class request
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    switch (req->bRequest)
    {
        case DFU_DNLOAD:
            DFU_Req_DNLOAD(pudev, req);
            break;

        case DFU_UPLOAD:
            DFU_Req_UPLOAD(pudev, req);
            break;

        case DFU_GETSTATUS:
            DFU_Req_GETSTATUS(pudev);
            break;

        case DFU_CLRSTATUS:
            DFU_Req_CLRSTATUS(pudev);
            break;

        case DFU_GETSTATE:
            DFU_Req_GETSTATE(pudev);
            break;

        case DFU_ABORT:
            DFU_Req_ABORT(pudev);
            break;

        case DFU_DETACH:
            DFU_Req_DETACH(pudev, req);
            break;

        default:
            USBD_EnumError(pudev, req);
            return USBD_FAIL;
    }

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USB_CtlTx (pudev, (uint8_t *)&USBD_DFU_AltSet, 1);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
    {
        USBD_DFU_AltSet = (uint8_t)(req->wValue);
    }
    else
    {
        USBD_EnumError (pudev, req);
    }

    return USBD_OK;
}

/**
  * @brief  Handle data IN stage in control endpoint 0
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_EP0_TxSent (void *pudev)
{
    uint32_t Addr;

    if (DeviceStatus[4] == STATE_dfuDNBUSY)
    {
        /* Decode the special command */
        if (BlockNum == 0)
        {
            if ((MAL_Buffer[0] == GET_COMMANDS) && (Length == 1))
            {}
            else if ((MAL_Buffer[0] == SET_ADDRESS_POINTER) && (Length == 5))
            {
                BaseAddress  = MAL_Buffer[1];
                BaseAddress |= MAL_Buffer[2] << 8;
                BaseAddress |= MAL_Buffer[3] << 16;
                BaseAddress |= MAL_Buffer[4] << 24;
            }
            else if ((MAL_Buffer[0] == ERASE) && (Length == 5))
            {
                BaseAddress  = MAL_Buffer[1];
                BaseAddress |= MAL_Buffer[2] << 8;
                BaseAddress |= MAL_Buffer[3] << 16;
                BaseAddress |= MAL_Buffer[4] << 24;
                DFU_MAL_Erase(BaseAddress);
            }
            else
            {
                USBD_EnumError (pudev, NULL);
            }
        }
        /* Regular download command */
        else if (BlockNum > 1)
        {
            /* Decode the required address */
            Addr = (BlockNum - 2) * TRANSFER_SIZE + BaseAddress;

            /* Preform the write operation */
            DFU_MAL_Write(Addr, Length);

            BlockNum = 0;
        }

        Length = 0;

        /* Update the device state and poll timeout */
        DeviceStatus[4] = STATE_dfuDNLOAD_SYNC;
        DeviceStatus[1] = 0;
        DeviceStatus[2] = 0;
        DeviceStatus[3] = 0;

        return USBD_OK;
    }
    else if (DeviceStatus[4] == STATE_dfuMANIFEST)/* Manifestation in progress */
    {
        /* Start leaving DFU mode */
        DFU_LeaveDFUMode(pudev);
    }

    return USBD_OK;
}

/**
  * @brief  Handle data OUT stage in control endpoint 0
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBD_DFU_EP0_RxReady (void *pudev)
{
    return USBD_OK;
}

/**
  * @brief  Handle the DFU_DETACH request
  * @param  pudev: pointer to usb device instance
  * @param  req: DFU class request
  * @retval None.
  */
static void DFU_Req_DETACH(void *pudev, USB_DEVICE_REQ *req)
{
    switch (DeviceStatus[4])
    {
        case STATE_dfuIDLE:
        case STATE_dfuDNLOAD_SYNC:
        case STATE_dfuDNLOAD_IDLE:
        case STATE_dfuMANIFEST_SYNC:
        case STATE_dfuUPLOAD_IDLE:
            DeviceStatus[0] = STATUS_OK;
            DeviceStatus[1] = 0;
            DeviceStatus[2] = 0;
            DeviceStatus[3] = 0; /* bwPollTimeout = 0ms */
            DeviceStatus[4] = STATE_dfuIDLE;
            DeviceStatus[5] = 0; /* iString */

            BlockNum = 0;
            Length = 0;
            break;

        default:
            break;
    }

    /* Check the detach capability in the DFU functional descriptor */
    if ((USBD_DFU_CfgDesc[12 + (9 * USBD_ITF_MAX_NUM)]) & DFU_DETACH_MASK)
    {
        /* Perform an Attach-Detach operation on USB bus */
        USBD_Disconnect(pudev);
        USBD_Connect(pudev);
    }
    else
    {
        /* Wait for the period of time specified in detach request */
        USB_HWP_mDelay (req->wValue);
    }
}

/**
  * @brief  Handle the DFU_DNLOAD request
  * @param  pudev: pointer to usb device instance
  * @param  req: DFU class request
  * @retval None
  */
static void DFU_Req_DNLOAD(void *pudev, USB_DEVICE_REQ *req)
{
    /* Data setup request */
    if (req->wLength > 0)
    {
        if ((DeviceStatus[4] == STATE_dfuIDLE) || (DeviceStatus[4] == STATE_dfuDNLOAD_IDLE))
        {
            /* Update the global length and block number */
            BlockNum = req->wValue;
            Length = req->wLength;

            DeviceStatus[4] = STATE_dfuDNLOAD_SYNC;

            /* Enable EP0 prepare receive the buffer */
            USB_CtlRx (pudev, (uint8_t*)MAL_Buffer, Length);
        }
        /* Unsupported state */
        else
        {
            USBD_EnumError (pudev, req);
        }
    }
    else
    {
        /* End of DNLOAD operation*/
        if ((DeviceStatus[4] == STATE_dfuDNLOAD_IDLE) || (DeviceStatus[4] == STATE_dfuIDLE))
        {
            Manifest_State = Manifest_In_Progress;
            DeviceStatus[1] = 0;
            DeviceStatus[2] = 0;
            DeviceStatus[3] = 0;
            DeviceStatus[4] = STATE_dfuMANIFEST_SYNC;
        }
        else
        {
            USBD_EnumError (pudev, req);
        }
    }
}

/**
  * @brief  Handles the DFU UPLOAD request.
  * @param  pudev: pointer to usb device instance
  * @param  req: DFU class request
  * @retval None
  */
static void  DFU_Req_UPLOAD (void *pudev, USB_DEVICE_REQ *req)
{
    uint8_t *Phy_Addr = NULL;
    uint32_t Addr = 0;

    /* Data setup request */
    if (req->wLength > 0)
    {
        if ((DeviceStatus[4] == STATE_dfuIDLE) || (DeviceStatus[4] == STATE_dfuUPLOAD_IDLE))
        {
            /* Update the global length and block number */
            BlockNum = req->wValue;
            Length = req->wLength;

            /* Set device poll timeout */
            DeviceStatus[1] = 0;
            DeviceStatus[2] = 0;
            DeviceStatus[3] = 0;

            /* DFU Get Command */
            if (BlockNum == 0)
            {
                /* Update the state machine */
                DeviceStatus[4] = (Length > 3) ? STATE_dfuIDLE : STATE_dfuUPLOAD_IDLE;

                /* Store the values of all supported commands */
                MAL_Buffer[0] = GET_COMMANDS;
                MAL_Buffer[1] = SET_ADDRESS_POINTER;
                MAL_Buffer[2] = ERASE;

                /* Send the status data over EP0 */
                USB_CtlTx (pudev, (uint8_t *)(&(MAL_Buffer[0])), 3);
            }
            else if (BlockNum > 1)
            {
                DeviceStatus[4] = STATE_dfuUPLOAD_IDLE;

                /* Change is Accelerated */
                Addr = (BlockNum - 2) * TRANSFER_SIZE + BaseAddress;

                /* Return the physical address where data are stored */
                Phy_Addr = DFU_MAL_Read(Addr, Length);

                /* Send the status data over EP0 */
                USB_CtlTx (pudev, Phy_Addr, Length);
            }
            else
            {
                DeviceStatus[4] = STATUS_errSTALLEDPKT;

                USBD_EnumError (pudev, req);
            }
        }
        else
        {
            Length = 0;
            BlockNum = 0;

            USBD_EnumError (pudev, req);
        }
    }
    else
    {
        DeviceStatus[1] = 0;
        DeviceStatus[2] = 0;
        DeviceStatus[3] = 0;
        DeviceStatus[4] = STATE_dfuIDLE;
    }
}

/**
  * @brief  Handle the DFU_GETSTATUS request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  DFU_Req_GETSTATUS (void *pudev)
{
    switch (DeviceStatus[4])
    {
        case STATE_dfuDNLOAD_SYNC:
            if (Length != 0)
            {
                DeviceStatus[4] = STATE_dfuDNBUSY;
                if (BlockNum == 0)
                {
                    if(MAL_Buffer[0] == ERASE)
                    {
                        DFU_MAL_GetStatus(BaseAddress, CMD_ERASE, DeviceStatus);
                    }
                    else
                    {
                        DFU_MAL_GetStatus(BaseAddress, CMD_WRITE, DeviceStatus);
                    }
                }
            }
            else
            {
                DeviceStatus[4] = STATE_dfuDNLOAD_IDLE;
                DeviceStatus[1] = 0;
                DeviceStatus[2] = 0;
                DeviceStatus[3] = 0;
            }
            break;

        case STATE_dfuMANIFEST_SYNC:
            if (Manifest_State == Manifest_In_Progress)
            {
                DeviceStatus[4] = STATE_dfuMANIFEST;
                DeviceStatus[1] = 1;
                DeviceStatus[2] = 0;
                DeviceStatus[3] = 0; /* bwPollTimeout = 1ms */
            }
            else if ((Manifest_State == Manifest_complete) && \
                (USBD_DFU_CfgDesc[11 + 9 * USBD_ITF_MAX_NUM] & 0x04))
            {
                DeviceStatus[4] = STATE_dfuIDLE;
                DeviceStatus[1] = 0;
                DeviceStatus[2] = 0;
                DeviceStatus[3] = 0;
            }
            break;

        default :
            break;
    }

    /* Send the status data of DFU interface to host over EP0 */
    USB_CtlTx (pudev, (uint8_t *)(&(DeviceStatus[0])), 6);
}

/**
  * @brief  Handle the DFU_CLRSTATUS request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  DFU_Req_CLRSTATUS (void *pudev)
{
    if (DeviceStatus[4] == STATE_dfuERROR)
    {
        DeviceStatus[0] = STATUS_OK;
        DeviceStatus[4] = STATE_dfuIDLE;
    }
    else
    {
        /* State Error */
        DeviceStatus[0] = STATUS_errUNKNOWN;
        DeviceStatus[4] = STATE_dfuERROR;
    }

    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0; /* bwPollTimeout = 0ms */
    DeviceStatus[5] = 0; /* iString: index = 0 */
}

/**
  * @brief  Handle the DFU_GETSTATE request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  DFU_Req_GETSTATE (void *pudev)
{
    /* Send the current state of the DFU interface to host */
    USB_CtlTx (pudev, &DeviceStatus[4], 1);
}

/**
  * @brief  Handle the DFU_ABORT request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  DFU_Req_ABORT (void *pudev)
{
    switch (DeviceStatus[4])
    {
        case STATE_dfuIDLE:
        case STATE_dfuDNLOAD_SYNC:
        case STATE_dfuDNLOAD_IDLE:
        case STATE_dfuMANIFEST_SYNC:
        case STATE_dfuUPLOAD_IDLE:
            DeviceStatus[0] = STATUS_OK;
            DeviceStatus[1] = 0;
            DeviceStatus[2] = 0;
            DeviceStatus[3] = 0; /* bwPollTimeout = 0ms */
            DeviceStatus[4] = STATE_dfuIDLE;
            DeviceStatus[5] = 0; /* iString: index = 0 */

            BlockNum = 0;
            Length = 0;
            break;

        default:
            break;
    }
}

/**
  * @brief  Leave DFU mode and reset device to jump to user loaded code
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
void  DFU_LeaveDFUMode (void *pudev)
{
    Manifest_State = Manifest_complete;

    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0; /* bwPollTimeout = 0ms */

    if ((USBD_DFU_CfgDesc[(11 + (9 * USBD_ITF_MAX_NUM))]) & 0x04)
    {
        DeviceStatus[4] = STATE_dfuMANIFEST_SYNC;
    }
    else
    {
        DeviceStatus[4] = STATE_dfuMANIFEST_WAIT_RESET;

        DFU_MAL_DeInit();

        /* Generate system reset to allow jumping to the user code */
        NVIC_SystemReset();
    }
}

/**
  * @brief  Get configuration descriptor
  * @param  USBSpeed: current device speed
  * @param  length: pointer to data length
  * @retval Pointer to descriptor buffer
  */
static uint8_t  *USBD_DFU_GetCfgDesc (uint8_t USBSpeed, uint16_t *len)
{
    *len = sizeof (USBD_DFU_CfgDesc);

    return (uint8_t*)USBD_DFU_CfgDesc;
}

/**
  * @brief  Get memory interfaces string descriptors
  * @param  USBSpeed: current device speed
  * @param  index: user string descriptor index
  * @param  len: pointer to data length
  * @retval Pointer to the string descriptor buffer or NULL if the descriptor is not supported
  */
static uint8_t*  USBD_DFU_GetUsrStringDesc (uint8_t USBSpeed, uint8_t index, uint16_t *len)
{
    /* Check if the requested interface string descriptor index is supported */
    if (index <= (USBD_INTERFACE_STR_IDX + USBD_ITF_MAX_NUM))
    {
        USBD_GetUnicodeString ((uint8_t *)USBD_DFU_StringDesc[index - USBD_INTERFACE_STR_IDX - 1], USBD_StrDesc, len);
        return USBD_StrDesc;
    }
    else
    {
        return NULL;
    }
}

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

/**
  * @}
  */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
