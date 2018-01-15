/**
  ******************************************************************************
  * @file    usbd_pwr.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Power management functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_pwr.h"

/* Private function prototypes ----------------------------------------------- */
static void  ResumeMCU (void);

#ifdef USB_DEVICE_LOW_PWR_MODE_SUPPORT
static void  LeaveLowPowerMode (void);
#endif

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */
  
/** @defgroup USBD_Power_Private_Variables
  * @{
  */
__IO RESUME_STATE State = RESUME_OFF;
__IO uint8_t  ESOFcount = 0;
__IO uint8_t  SuspendEnabled = 1;
__IO uint8_t  RemoteWakeupOn = 0;

/**
  * @}
  */

/** @defgroup USBD_Power_Private_Functions
  * @{
  */

/**
  * @brief  Usb wakeup first operation is to wakeup mcu
  * @param  None
  * @retval None
  */
static void  ResumeMCU (void)
{
    /* Clear low_power mode bit in CTLR */
    _SetCTLR(_GetCTLR() & (~CTLR_LOWM));

#ifdef USB_DEVICE_LOW_PWR_MODE_SUPPORT

    /* Restore normal operations */
    LeaveLowPowerMode();

#endif

    /* Clear SETSPS bit */
    _SetCTLR(_GetCTLR() & (~CTLR_SETSPS));
}

#ifdef USB_DEVICE_LOW_PWR_MODE_SUPPORT

/**
  * @brief  Restores system clocks and power while exiting suspend mode
  * @param  None
  * @retval None
  */
static void  LeaveLowPowerMode (void)
{
    /* Restore system clock */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    while(RCC_GetBitState(RCC_FLAG_HSESTB) == RESET);

    /* Enable PLL */
    RCC_PLL_Enable(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetBitState(RCC_FLAG_PLLSTB) == RESET);

    /* Select PLL as system clock source */
    RCC_CK_SYSConfig(RCC_SYSCLKSOURCE_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetCK_SYSSource() != 0x08);

    /* Low power sleep on exit disabled */
    NVIC_SystemLowPowerConfig(NVIC_LOWPOWER_SLEEPONEXIT, DISABLE);
}

#endif

/**
  * @brief  Set usb device to suspend mode
  * @param  None
  * @retval None
  */
void  USBD_Suspend (void)
{
    /* Set suspend and low-power mode in the macrocell */
    _SetCTLR(_GetCTLR() | CTLR_SETSPS | CTLR_LOWM);

#ifdef USB_DEVICE_LOW_PWR_MODE_SUPPORT

    /* When wakeup flag in not set, enter system to DEEP_SLEEP mode */
    if((_GetIFR() & IFR_WKUPIF) == 0)
    {
        /* Enter DEEP_SLEEP mode with LDO in low power mode */
        PWR_DEEPSLEEPMode_Entry(PWR_LDO_LOWPOWER, PWR_DEEPSLEEPENTRY_WFI);
    }
    else
    {
        /* Clear wakeup flag */
        _SetIFR(CLR_WKUPIF);

        /* Clear SETSPS to abort entry in suspend mode */
        _SetCTLR(_GetCTLR() & (~CTLR_SETSPS));
    }

#endif
}

/**
  * @brief  Resume state machine handling
  * @param  ResumeValue: a state machine value (RESUME_STATE)
  *   This parameter can be any one of the following values:
  *     @arg RESUME_EXTERNAL
  *     @arg RESUME_INTERNAL
  *     @arg RESUME_LATER
  *     @arg RESUME_ON
  *     @arg RESUME_OFF
  *     @arg RESUME_ESOF: it doesn't change state just allowing
  *       decrementing of the ESOF counter in different states.
  * @retval None
  */
void  USBD_Resume (RESUME_STATE ResumeValue)
{
    if (ResumeValue != RESUME_ESOF) State = ResumeValue;

    switch (State)
    {
        case RESUME_EXTERNAL:
            if (RemoteWakeupOn == 0)
            {
                ResumeMCU();
                State = RESUME_OFF;
            }
            else
            {
                /* If resume is detected during the RemoteWakeup signalling, need keep RemoteWakeup handling */
                State = RESUME_ON;
            }
            break;

        case RESUME_INTERNAL:
            ResumeMCU();
            RemoteWakeupOn = 1;

        case RESUME_LATER:
            ESOFcount = 15;
            _SetCTLR(_GetCTLR() | CTLR_RSREQ);
            State = RESUME_ON;
            break;

        case RESUME_ON:
            ESOFcount--;
            if (ESOFcount == 0)
            {
                _SetCTLR(_GetCTLR() & (~CTLR_RSREQ));
                State = RESUME_OFF;
                RemoteWakeupOn = 0;
            }
            break;

        case RESUME_OFF:
        case RESUME_ESOF:
        default:
            break;
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

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
