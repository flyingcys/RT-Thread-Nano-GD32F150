/**
  ******************************************************************************
  * @file    usb_core.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB device driver interface layer used to access the core
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Driver
  * @{
  */

/** @defgroup USB_Core_Private_Variables
  * @{
  */
uint32_t InterruptMask = 0;

/**
  * @}
  */

/** @defgroup USB_Core_Private_functions
  * @{
  */

/**
  * @brief  Device register initialization
  * @param  None
  * @retval None
  */
void  DR_Init (void)
{
    /* Just reset the CLOSE bit */
    _SetCTLR(CTLR_SETRST);

    /* May be need wait some time(tSTARTUP) ... */

    /* Clear SETRST bit in CTLR register */
    _SetCTLR(0);

    /* Clear all pending interrupts */
    _SetIFR(0);

    /* Set allocation buffer address */
    _SetBAR(BUFFER_ADDRESS);

    InterruptMask = IER_MASK;
  
#ifdef LPM_ENABLED
    /* enable L1REQ interrupt */
    _SetLPM_CNTR(LPM_STIE);
#endif

    /* Enable all interrupts mask bits */
    _SetCTLR(InterruptMask); 
}

/**
  * @brief  Device register configure when stop device
  * @param  None
  * @retval None
  */
void  DR_StopDevice (void)
{
    /* Disable all interrupts and set USB reset */
    _SetCTLR(CTLR_SETRST);

    /* Clear all interrupt flags */
    _SetIFR(0);

    /* Close device */
    _SetCTLR(CTLR_SETRST | CTLR_CLOSE);  
}

/**
  * @brief  Configure buffer for endpoint
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @param  EpKind: endpoint kind
  *   This parameter can be any one of the following values:
  *     @arg USB_SNG_BUFTYPE: single buffer will be used
  *     @arg USB_DBL_BUFTYPE: double buffer will be used
  * @param  BufAddr: endpoint address in the allocation buffer:
  *                   If the endpoint use single buffer
  *                   this parameter is a 16-bit value providing the bufaddress allocated to endpoint.
  *                   If the endpoint use double buffer
  *                   this parameter is a 32-bit value providing the endpoint buffer 0 address
  *                   in the LSB part and endpoint buffer 1 address in the MSB part.
  * @retval None
  */
void  USB_EP_BufConfig (USB_CORE_HANDLE *pudev,
                                uint8_t EpAddr,
                                uint8_t EpKind,
                                uint32_t BufAddr)

{
    USB_EP *ep;
    uint8_t EpID = EpAddr & 0x7F;

    if (EpAddr >> 7)
    {
        /* Get an IN endpoint */
        ep = &pudev->dev.in_ep[EpID];
    }
    else
    {
        /* Get a OUT endpoint */
        ep = &pudev->dev.out_ep[EpID];
    }

    if (EpKind == USB_SNG_BUFTYPE)
    {
        /* Endpoint is single buffer kind */
        ep->is_dblbuf = 0;

        /* Configure the buffer address */
        ep->bufaddress = (uint16_t)BufAddr;
    }
    else if(EpKind == USB_DBL_BUFTYPE)
    {
        /* Endpoint is double buffer kind */
        ep->is_dblbuf = 1;

        /* Get the buffer0 and buffer1 address */
        ep->buf0addr = BufAddr & 0xFFFF;
        ep->buf1addr = (BufAddr & 0xFFFF0000) >> 16;

        /* Set the endpoint kind as double buffer */
        _SetEPDoubleBuff(EpID);

        /* Set buffer address for double buffered endpoint */
        _SetEPDblBufAddr(EpID, ep->buf0addr, ep->buf1addr);
    }
}


/**
  * @brief  Endpoint initialization
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @param  EpType: endpoint type
  *   This parameter can be any one of the following values:
  *     @arg USB_EPTYPE_CONTROL: control transfer endpoint
  *     @arg USB_EPTYPE_BULK: bulk transfer endpoint
  *     @arg USB_EPTYPE_INT: interrupt transfer endpoint
  *     @arg USB_EPTYPE_ISOC: isochronous transfer endpoint
  * @param  EpMps: endpoint max packet size
  * @retval None
  */
void  USB_EP_Init (USB_CORE_HANDLE *pudev,
                          uint8_t EpAddr,
                          uint8_t EpType,
                          uint16_t EpMps)
{
    USB_EP *ep;
    uint8_t EpID = EpAddr & 0x7F;

    /* Set the endpoint type */
    switch (EpType)
    {
        case USB_EPTYPE_CONTROL:
            _SetEPType(EpID, EP_CONTROL);
            break;

        case USB_EPTYPE_BULK:
            _SetEPType(EpID, EP_BULK);
            break;

        case USB_EPTYPE_INT:
            _SetEPType(EpID, EP_INTERRUPT);
            break;

        case USB_EPTYPE_ISOC:
            _SetEPType(EpID, EP_ISO);
            break;
    }

    if (EpAddr >> 7)
    {
        ep = &pudev->dev.in_ep[EpID];

        /* Initialize the transmit endpoint data toggle bit */
        _ClearDTG_TX(EpID);

        if(ep->is_dblbuf == 0)
        {
            /* Set the endpoint transmit buffer address */
            _SetEPTxAddr(EpID, ep->bufaddress);

            /* Configure the endpoint status as NAK status */
            _SetEPTxStatus(EpID, EPTX_NAK);

            /* Control endpoint need toggle data toggle bit */
            if(EpID == 0) _ToggleDTG_TX(EpID);
        }
        else
        {
            /* Clear another direction data toggle bit when use double buffer */
            _ClearDTG_RX(EpID);

            /* Set the Double buffer counter */
            _SetEPDblBuffCount(EpID, DBUF_EP_IN, 0);

            /* Configure the double buffer endpoint status */
            _SetEPRxStatus(EpID, EPRX_DISABLED);
            _SetEPTxStatus(EpID, EPTX_VALID);
        }
    }
    else
    {
        ep = &pudev->dev.out_ep[EpID];

        _ClearDTG_RX(EpID);

        if(ep->is_dblbuf == 0)
        {
            /*Set the endpoint receive buffer address */
            _SetEPRxAddr(EpID, ep->bufaddress);

            /* Configure the endpoint status as NAK */
            _SetEPRxStatus(EpID, EPRX_NAK);
        }
        else
        {
            _ClearDTG_TX(EpID);

            /* Toggle the SW_BUF bit(TX data toggle bit) */
            _ToggleDTG_TX(EpID);

            /* Configure the endpoint status as DISABLED in two direction */
            _SetEPRxStatus(EpID, EPRX_DISABLED);
            _SetEPTxStatus(EpID, EPTX_DISABLED);
        }
    }

    /* Initialize the basic parameters */
    ep->maxpacket = EpMps;
    ep->is_stall = 0;

    /* Initialize the transaction level parameters */
    ep->xfer_buf = 0;
    ep->xfer_len = 0;
    ep->xfer_count = 0;
    ep->ctrl_count = 0;
}

/**
  * @brief  Configure the endpoint when it is disabled
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @retval None
  */
void  USB_EP_DeInit (USB_CORE_HANDLE *pudev, uint8_t EpAddr)
{
    USB_EP *ep;
    uint8_t EpID = EpAddr & 0x7F;

    if (EpAddr >> 7)
    {
        ep = &pudev->dev.in_ep[EpID];

        _ClearDTG_TX(EpID);

        if(ep->is_dblbuf != 0)
        {
            /* Clear the data toggle bits of the endpoint */
            _ClearDTG_RX(EpID);

            /* Reset value of the data toggle bits for the endpoint IN */
            _ToggleDTG_TX(EpID);

            /* Configure the endpoint status as DISABLED */
            _SetEPRxStatus(EpID, EPRX_DISABLED);
        }

        /* Configure the endpoint status as DISABLED */
        _SetEPTxStatus(EpID, EPTX_DISABLED); 
    }
    else
    {
        ep = &pudev->dev.out_ep[EpID];

        _ClearDTG_RX(EpID);

        if(ep->is_dblbuf != 0)
        {
            /* Clear the data toggle bits of the endpoint */
            _ClearDTG_TX(EpID);

            /* Reset value of the data toggle bits for the endpoint OUT */
            _ToggleDTG_RX(EpID);

            /* Configure the endpoint status as DISABLED */
            _SetEPTxStatus(EpID, EPTX_DISABLED);
        }

        /* Configure the endpoint status as DISABLED */
        _SetEPRxStatus(EpID, EPRX_DISABLED);
    }
}


/**
  * @brief  Endpoint prepare to receive data
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @param  pbuf: user buffer address pointer
  * @param  BufLen: buffer length
  * @retval None
  */
void  USB_EP_Rx (USB_CORE_HANDLE *pudev,
                        uint8_t  EpAddr,
                        uint8_t *pbuf,
                        uint16_t BufLen)
{
    USB_EP *ep;
    uint8_t EpID = EpAddr & 0x7F;

    ep = &pudev->dev.out_ep[EpID];

    /* Configure the transaction level parameters */
    ep->xfer_buf = pbuf;
    ep->xfer_len = BufLen;

    if (ep->is_dblbuf == 0) 
    {
        /* Set receive buffer byte count */
        _SetEPRxCount(EpID, ep->maxpacket);
    }
    else
    {
        /* Set the double buffer receive byte count */
        _SetEPDblBuffCount(EpID, DBUF_EP_OUT, ep->maxpacket);
    }

    /* Enable endpoint to receive */
    _SetEPRxStatus(EpID, EPRX_VALID);
}

/**
  * @brief  Endpoint prepare to transmit data
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @param  pbuf: transmit buffer address pointer
  * @param  BufLen: buffer length
  * @retval None
  */
void  USB_EP_Tx (USB_CORE_HANDLE *pudev,
                        uint8_t  EpAddr,
                        uint8_t *pbuf,
                        uint16_t BufLen)
{
    USB_EP *ep;
    __IO uint32_t len = 0;
    uint8_t EpID = EpAddr & 0x7F;

    ep = &pudev->dev.in_ep[EpID];

    /* Configure the transaction level parameters */
    ep->xfer_buf = pbuf;
    ep->xfer_len = BufLen;
    ep->xfer_count = 0;

    /* Transmit length is more than one packet */
    if (ep->xfer_len > ep->maxpacket)
    {
        len = ep->maxpacket;
    }
    else
    {
        len = ep->xfer_len;
    }

    if (ep->is_dblbuf == 0)
    {
        UserCopyToBuffer(ep->xfer_buf, ep->bufaddress, len);
        _SetEPTxCount(EpID, len);
    }
    else
    {
        /* Set the Double buffer counter */
        _SetEPDblBuffCount(EpID, DBUF_EP_IN, len);

        /* Copy the data to the double buffered endpoint buffer */
        if (_GetEPxCSR(EpID) & EPTX_DTG)
        {
            UserCopyToBuffer(ep->xfer_buf, ep->buf1addr, len);
        }
        else
        {
            UserCopyToBuffer(ep->xfer_buf, ep->buf0addr, len);
        }

        FreeUserBuffer(EpID, DBUF_EP_IN);
    }

    /* Enable endpoint to transmit */
    _SetEPTxStatus(EpID, EPTX_VALID);
}

/**
  * @brief  Set an endpoint to STALL status
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @retval None
  */
void  USB_EP_Stall (USB_CORE_HANDLE *pudev, uint8_t EpAddr)
{
    USB_EP *ep;
    uint8_t EpID = EpAddr & 0x7F;

    if (EpAddr >> 7)
    {
        ep = &pudev->dev.in_ep[EpID]; 
        _SetEPTxStatus(EpID, EPTX_STALL);
    }
    else
    {
       ep = &pudev->dev.out_ep[EpID];
       _SetEPRxStatus(EpID, EPRX_STALL);
    }

    if (EpID == 0)
    {
        /* Control endpoint need to be stalled in two directions */
        _SetEPRxTxStatus(EpID, EPRX_STALL, EPTX_STALL); 
    }

    /* Endpoint now is stalled */
    ep->is_stall = 1;
}

/**
  * @brief  Clear endpoint stalled status
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @retval None
  */
void  USB_EP_ClrStall (USB_CORE_HANDLE *pudev, uint8_t EpAddr)
{
    USB_EP *ep;
    uint8_t EpID = EpAddr & 0x7F;

    if (EpAddr >> 7)
    {
        ep = &pudev->dev.in_ep[EpID];
        _ClearDTG_TX(EpID);
        _SetEPTxStatus(EpID, EPTX_VALID);
    }
    else
    {
        ep = &pudev->dev.out_ep[EpID];
        _ClearDTG_RX(EpID);
        _SetEPRxStatus(EpID, EPRX_VALID);
    }

    /* Endpoint now is not stalled */
    ep->is_stall = 0;
}

/**
  * @brief  Set USB device and endpoints address
  * @param  pudev: pointer to usb core instance
  * @param  Addr: device address to set
  * @retval None
  */
void  USB_EP_SetAddress (USB_CORE_HANDLE *pudev, uint8_t Addr)
{
    uint8_t i;

    /* Set endpoints address */
    for (i = 0; i < EP_COUNT; i++)
    {
        _SetEPAddress(i, i);
    }

    /* Set device address and enable USB module */
    _SetAR(Addr | AR_USBEN); 
}

/**
  * @brief  Get the endpoint status
  * @param  pudev: pointer to usb core instance
  * @param  EpAddr: endpoint address
  *   In this parameter:
  *     bit0..bit6: endpoint number which is in (0..7)
  *     bit7: endpoint direction which can be IN(1) or OUT(0)
  * @retval Endpoint status
  */
uint8_t  USB_EP_GetStatus (USB_CORE_HANDLE *pudev, uint8_t EpAddr)
{
    if (EpAddr >> 7)
    {
        return _GetEPTxStatus((EpAddr & 0x7F));
    }
    else
    {
        return _GetEPRxStatus(EpAddr);
    }
}

/**
  * @brief  Transmit data on the control pipe
  * @param  pudev: pointer to usb core instance
  * @param  pbuf: data buffer pointer
  * @param  Len: data length
  * @retval usb operation status
  */
uint8_t  USB_CtlTx (USB_CORE_HANDLE  *pudev,
                              uint8_t *pbuf,
                              uint16_t Len)
{ 
    pudev->dev.device_cur_state = USB_CTRL_DATA_IN;

    USB_EP_Tx(pudev, EP0_IN, pbuf, Len);

    return USB_OK;
}

/**
  * @brief  Continue transmitting data on the control pipe
  * @param  pudev: pointer to usb core instance
  * @param  pbuf: data buffer pointer
  * @param  Len: data length
  * @retval usb operation status
  */
uint8_t  USB_CtlContinueTx (USB_CORE_HANDLE  *pudev,
                                      uint8_t *pbuf,
                                      uint16_t Len)
{
    USB_EP_Tx (pudev, EP0_IN, pbuf, Len);

    return USB_OK;
}

/**
  * @brief  Receive data on the control pipe
  * @param  pudev: pointer to usb core instance
  * @param  pbuf: pointer to data buffer
  * @param  Len: length of data to be received
  * @retval usb operation status
  */
uint8_t  USB_CtlRx (USB_CORE_HANDLE  *pudev,
                              uint8_t *pbuf,
                              uint16_t Len)
{
    pudev->dev.device_cur_state = USB_CTRL_DATA_OUT;

    USB_EP_Rx(pudev, EP0_OUT, pbuf, Len);

    return USB_OK;
}

/**
  * @brief  Continue receive data on the contrl pipe
  * @param  pudev: pointer to usb core instance
  * @param  pbuf: data buffer pointer
  * @param  Len: data length
  * @retval usb operation status
  */
uint8_t  USB_CtlContinueRx (USB_CORE_HANDLE  *pudev,
                                      uint8_t *pbuf,
                                      uint16_t Len)
{
    USB_EP_Rx(pudev, EP0_OUT, pbuf, Len);

    return USB_OK;
}

/**
  * @brief  Transmit status stage on the control pipe
  * @param  pudev: pointer to usb core instance
  * @retval usb operation status
  */
uint8_t  USB_CtlTransmitStatus (USB_CORE_HANDLE *pudev)
{
    pudev->dev.device_cur_state = USB_CTRL_STATUS_IN;

    USB_EP_Tx(pudev, EP0_IN, NULL, 0);

    return USB_OK;
}

/**
  * @brief  Receive status stage on the control pipe
  * @param  pudev: pointer to usb core instance
  * @retval usb operation status
  */
uint8_t  USB_CtlReceiveStatus (USB_CORE_HANDLE *pudev)
{
    _Set_Status_Out(EP0);

    pudev->dev.device_cur_state = USB_CTRL_STATUS_OUT;

    USB_EP_Rx(pudev, EP0_OUT, NULL, 0); 

    return USB_OK;
}

/**
  * @brief  Get the received data length
  * @param  pudev: pointer to usb core instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval Received data length
  */
uint16_t  USB_GetRxCount (USB_CORE_HANDLE *pudev, uint8_t EpID)
{
    return pudev->dev.out_ep[EpID].xfer_count;
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

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
