/**
  ******************************************************************************
  * @file    usb_int.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB device interrupt routines
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_int.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Driver
  * @{
  */

/** @defgroup USB_Interrupt_Private_Variables
  * @{
  */
extern uint32_t InterruptMask;
extern USB_CORE_HANDLE USB_Device_dev;
extern USB_INTHandler_TypeDef *USB_INT_fops;

#ifdef LPM_ENABLED
__IO uint32_t  L1_remote_wakeup = 0;
__IO uint32_t  BESL = 0;
__IO uint16_t  LPM_Istr = 0;
#endif

/**
  * @}
  */

/** @defgroup USB_Interrupt_Private_functions
  * @{
  */

/**
  * @brief  USB interrupt events service routine
  * @param  None
  * @retval None
  */
void  USB_Ifr (void)
{
    __IO uint16_t Ifr = 0; 
    __IO uint16_t Ctlr = 0;

    Ifr = _GetIFR();

#ifdef LPM_ENABLED 
    LPM_Istr = GetLPM_ISTR(); 
#endif

    if (Ifr & IFR_STIF & InterruptMask)
    {
        /* The endpoint successful transfer interrupt service */
        USB_INT_fops->LP_ST(&USB_Device_dev);
    }

    if (Ifr & IFR_RSTIF & InterruptMask)
    {
        /* Clear reset interrupt flag in IFR */
        _SetIFR((uint16_t)CLR_RSTIF);

        /* USB reset interrupt handle */
        USB_INT_fops->Reset(&USB_Device_dev);
    }

#if (IER_MASK & IFR_PMOUIF)
    if (Ifr & IFR_PMOUIF & InterruptMask)
    {
        /* Clear packet memory overrun/underrun interrupt flag in IFR */
        _SetIFR((uint16_t)CLR_PMOUIF);

        /* USB packet memory overrun/underrun interrrupt handle */
        USB_INT_fops->PMOU(&USB_Device_dev);
    }
#endif

#if (IER_MASK & IFR_ERRIF)
    if (Ifr & IFR_ERRIF & InterruptMask)
    {
        /* Clear error interrupt flag in IFR */
        _SetIFR((uint16_t)CLR_ERRIF);

        /* USB error interrupt handle */
        USB_INT_fops->Error(&USB_Device_dev);
    }
#endif

#if (IER_MASK & IFR_WKUPIF)
    if (Ifr & IFR_WKUPIF & InterruptMask)
    {
        /* Clear wakeup interrupt flag in IFR */
        _SetIFR((uint16_t)CLR_WKUPIF);

        /* USB wakeup interrupt handle */
        USB_INT_fops->WakeUp(&USB_Device_dev);

#ifdef LPM_ENABLED
    /* clear L1 remote wakeup flag */
    L1_remote_wakeup = 0;
#endif
    }
#endif

#if (IER_MASK & IFR_SPSIF)
    if (Ifr & IFR_SPSIF & InterruptMask)
    {
        if(!(_GetCTLR() & CTLR_RSREQ))
        {
            /* Process library core layer suspend routine*/
            USB_INT_fops->Suspend(&USB_Device_dev);

            /* Clear of suspend interrupt flag bit must be done after setting of CTLR_SETSPS */
            _SetIFR((uint16_t)CLR_SPSIF);
        }
    }
#endif

#if (IER_MASK & IFR_SOFIF)
    if (Ifr & IFR_SOFIF & InterruptMask)
    {
        /* clear SOF interrupt flag in IFR */
        _SetIFR((uint16_t)CLR_SOFIF);

        /* USB SOF interrupt handle */
        USB_INT_fops->SOF(&USB_Device_dev);
    }

    if (Ifr & IFR_ESOFIF & InterruptMask)
    {
        /* clear ESOF interrupt flag in IFR */
        _SetIFR((uint16_t)CLR_ESOFIF);

        /* USB ESOF interrupt handle */
        USB_INT_fops->ESOF(&USB_Device_dev);
    }
#endif

#ifdef LPM_ENABLED
    if (LPM_Istr & _GetLPM_CNTR())
    {
        /* Clear L1 ST flag in LPM ISTR */
        _SetLPM_ISTR(LPM_CLR_STIF);

        /* Read BESL field from subendpoint0 register which coressponds to HIRD parameter in LPM spec */
        BESL = (_GetSUBEP0R() & ATTR_HIRD) >> 4;

        /* Read BREMOTEWAKE bit from subendpoint0 register which corresponding to bRemoteWake bit in LPM request */
        L1_remote_wakeup = (_GetSUBEP0R() & ATTR_BREMOTEWAKE) >> 8;

        /* Process usb device core layer suspend routine */
        USB_INT_fops->Suspend(&USB_Device_dev); 

        /* Enter usb model in suspend and system in low power mode (DEEP_SLEEP mode) */
        Suspend();
    }
#endif
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
