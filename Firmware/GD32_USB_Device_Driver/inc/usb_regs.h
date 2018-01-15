/**
  ******************************************************************************
  * @file    usb_regs.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB cell registers definition and handle macros
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_REGS_H
#define __GD32F1X0_USB_REGS_H

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Driver
  * @{
  */

/** @defgroup USB_Exported_Types
  * @{
  */

/**
  * @brief USB device speed
  */
enum USB_SPEED
{
    USB_SPEED_LOW,
    USB_SPEED_FULL
};

/**
  * @brief USB double buffer endpoint direction
  */
typedef enum _DBUF_EP_DIR
{
    DBUF_EP_IN,
    DBUF_EP_OUT,
    DBUF_EP_ERR,
}
DBUF_EP_DIR;

/**
  * @}
  */

/** @defgroup USB_Register_Base_Address
  * @{
  */
#define REG_BASE                        (0x40005C00L)  /*!< Base address of USB IP peripheral registers */
#define PBA_ADDR                        (0x40006000L)  /*!< Base address of USB IP packet buffer area */

/**
  * @}
  */

/** @defgroup USB_Common_Register 
  * @{
  */
#define CTLR                            ((__IO unsigned *)(REG_BASE + 0x40))  /*!< Control register */
#define IFR                             ((__IO unsigned *)(REG_BASE + 0x44))  /*!< Interrupt flag register */
#define SR                              ((__IO unsigned *)(REG_BASE + 0x48))  /*!< Status register */
#define AR                              ((__IO unsigned *)(REG_BASE + 0x4C))  /*!< Device address register */
#define BAR                             ((__IO unsigned *)(REG_BASE + 0x50))  /*!< Buffer address register */

/**
  * @}
  */

/** @defgroup USB_LPM_Register
  * @{
  */
#define SUB_EP0REG                      ((__IO unsigned *)(REG_BASE + 0x100))  /*!< Sub endpoint0 register(for LPM) */
#define LPM_CNTR                        ((__IO unsigned *)(REG_BASE + 0x140))  /*!< LPM interrupt control register */
#define LPM_ISTR                        ((__IO unsigned *)(REG_BASE + 0x144))  /*!< LPM interrupt status register */

/**
  * @}
  */

/** @defgroup USB_Endpoint_Control_and_Status_Register
  * @{
  */
#define EP0CSR                          ((__IO unsigned *)(REG_BASE)) /*!< endpoint 0 control and status register address */

/**
  * @}
  */

/** @defgroup Endpoints_Address
  * @{
  */

/**
  * @brief First bit is direction(0 for Rx and 1 for Tx)
  */
#define EP0_OUT                         ((uint8_t)0x00)
#define EP0_IN                          ((uint8_t)0x80)
#define EP1_OUT                         ((uint8_t)0x01)
#define EP1_IN                          ((uint8_t)0x81)
#define EP2_OUT                         ((uint8_t)0x02)
#define EP2_IN                          ((uint8_t)0x82)
#define EP3_OUT                         ((uint8_t)0x03)
#define EP3_IN                          ((uint8_t)0x83)
#define EP4_OUT                         ((uint8_t)0x04)
#define EP4_IN                          ((uint8_t)0x84)
#define EP5_OUT                         ((uint8_t)0x05)
#define EP5_IN                          ((uint8_t)0x85)
#define EP6_OUT                         ((uint8_t)0x06)
#define EP6_IN                          ((uint8_t)0x86)
#define EP7_OUT                         ((uint8_t)0x07)
#define EP7_IN                          ((uint8_t)0x87)

/**
  * @}
  */

/** @defgroup Endpoints_Identifier
  * @{
  */
#define EP0                             ((uint8_t)0)
#define EP1                             ((uint8_t)1) 
#define EP2                             ((uint8_t)2)
#define EP3                             ((uint8_t)3)
#define EP4                             ((uint8_t)4)
#define EP5                             ((uint8_t)5)
#define EP6                             ((uint8_t)6)
#define EP7                             ((uint8_t)7)

/**
  * @}
  */

/** @defgroup USB_Registers_Bits_Definition
  * @{
  */

/**
  * @brief CTLR control register bits definitions
  */
#define CTLR_STIE                       (0x8000)  /*!< Successful transfer interrupt enable Mask */
#define CTLR_PMOUIE                     (0x4000)  /*!< Packet memory overrun/underrun interrupt enable Mask */
#define CTLR_ERRIE                      (0x2000)  /*!< Error interrupt enable Mask */
#define CTLR_WKUPIE                     (0x1000)  /*!< Wakeup interrupt enable Mask */
#define CTLR_SPSIE                      (0x0800)  /*!< Suspend state interrupt enable Mask */
#define CTLR_RSTIE                      (0x0400)  /*!< Reset interrupt enable Mask */
#define CTLR_SOFIE                      (0x0200)  /*!< Start of frame interrupt enable Mask */
#define CTLR_ESOFIE                     (0x0100)  /*!< Expected start of frame interrupt enable Mask */
#define CTLR_RSREQ                      (0x0010)  /*!< Resume request */
#define CTLR_SETSPS                     (0x0008)  /*!< Set suspend state */
#define CTLR_LOWM                       (0x0004)  /*!< Low-power mode at suspend state */
#define CTLR_CLOSE                      (0x0002)  /*!< Goes to close state */
#define CTLR_SETRST                     (0x0001)  /*!< Set USB reset */

/**
  * @brief IFR interrupt events bits definitions
  */
#define IFR_STIF                        (0x8000)  /*!< Successful transfer interrupt flag (read only bit) */
#define IFR_PMOUIF                      (0x4000)  /*!< Packet memory overrun/underrun interrupt flag (clear-only bit) */
#define IFR_ERRIF                       (0x2000)  /*!< Error interrupt flag (clear-only bit) */
#define IFR_WKUPIF                      (0x1000)  /*!< Wakeup interrupt flag (clear-only bit) */
#define IFR_SPSIF                       (0x0800)  /*!< Suspend state interrupt flag (clear-only bit) */
#define IFR_RSTIF                       (0x0400)  /*!< Reset interrupt flag (clear-only bit) */
#define IFR_SOFIF                       (0x0200)  /*!< Start of frame interrupt flag (clear-only bit) */
#define IFR_ESOFIF                      (0x0100)  /*!< Expected start of frame interrupt flag(clear-only bit) */
#define IFR_DIR                         (0x0010)  /*!< Direction of transaction (read-only bit) */
#define IFR_EPNUM                       (0x000F)  /*!< Endpoint number (read-only bit)  */

#define CLR_STIF                        (~IFR_STIF)    /*!< Clear Successful transfer flag */
#define CLR_PMOUIF                      (~IFR_PMOUIF)  /*!< Clear Packet memory overrun/underrun flag */
#define CLR_ERRIF                       (~IFR_ERRIF)   /*!< Clear Error flag */
#define CLR_WKUPIF                      (~IFR_WKUPIF)  /*!< Clear Wake up flag */
#define CLR_SPSIF                       (~IFR_SPSIF)   /*!< Clear Suspend state flag */
#define CLR_RSTIF                       (~IFR_RSTIF)   /*!< Clear Reset flag */
#define CLR_SOFIF                       (~IFR_SOFIF)   /*!< Clear Start of frame flag */
#define CLR_ESOFIF                      (~IFR_ESOFIF)  /*!< Clear Expected start of frame flag */

/**
  * @brief SR status register bit definitions
  */
#define SR_RXDP                         (0x8000)  /*!< Receive data + line status */
#define SR_RXDM                         (0x4000)  /*!< Receive data - line status */
#define SR_LOCK                         (0x2000)  /*!< Locked the USB */
#define SR_SOFLN                        (0x1800)  /*!< SOF lost number */
#define SR_FCNT                         (0x07FF)  /*!< Frame number count */

/**
  * @brief AR device address register bit definitions
  */
#define AR_USBEN                        (0x80)  /*!< USB module enable */
#define AR_USBADDR                      (0x7F)  /*!< USB device address */

/**
  * @brief Sub endpoint register bit definitions
  */
#define SUB_ST                          (0x8000)  /*!< LPM token successful receive */
#define SUB_STA                         (0x3000)  /*!< Status bits for the handshake of receiving subpid LPM */
#define SUBPID_ATTR                     (0x07ff)  /*!< LPM token bmattribute field */

#define ATTR_BREMOTEWAKE                (0x0100)  /*!< bRemoteWake bit in bmAttributes field */
#define ATTR_HIRD                       (0x00f0)  /*!< Host initiated resume duration field */
#define ATTR_BLINKSTATE                 (0x000f)  /*!< Link state */

/**
  * @brief STAT_TX[1:0] STATus for Tx transfer
  */
#define SUBEP_DISABLE                   (0x0000)  /*!< Ignore all LPM token reception requests of this endpoint */
#define SUBEP_STALL                     (0x1000)  /*!< STALL handshake status */
#define SUBEP_NYET                      (0x2000)  /*!< NYET handshake status */
#define SUBEP_VALID                     (0x3000)  /*!< Enable endpoint for reception LPM token */

/**
  * @brief LPM control register bit definitions
  */
#define LPM_STIE                        (0x8000)  /*!< LPM token successful transfer interrupt enable mask */

/**
  * @brief LPM interrupt status register bit definitions
  */
#define LPM_STIF                        (0x8000)   /*!< LPM token successful transfer interrupt flag */
#define LPM_CLR_STIF                    (~LPM_STIF)/*!< clear successful transfer interrupt flag */

/**
  * @brief EPCSR endpoint control and status register bit definitions
  */
#define EPRX_ST                         (0x8000)  /*!< EndPoint reception successful transferred */
#define EPRX_DTG                        (0x4000)  /*!< EndPoint reception data PID toggle */
#define EPRX_STA                        (0x3000)  /*!< EndPoint reception status bits */
#define EP_SETUP                        (0x0800)  /*!< EndPoint setup transaction completed */
#define EP_CTL                          (0x0600)  /*!< EndPoint type control */
#define EP_KCTL                         (0x0100)  /*!< EndPoint kind control */
#define EPTX_ST                         (0x0080)  /*!< EndPoint transmission successful transfer */
#define EPTX_DTG                        (0x0040)  /*!< EndPoint transmission data toggle */
#define EPTX_STA                        (0x0030)  /*!< EndPoint transmission transfers status bits */
#define EP_AR                           (0x000F)  /*!< EndPoint address */

/**
  * @brief Endpoint control and status register mask (no toggle fields)
  */
#define EPCSR_MASK                      (EPRX_ST|EP_SETUP|EP_CTL|EP_KCTL|EPTX_ST|EP_AR)

/**
  * @brief EP_CTL[1:0] endpoint type control
  */
#define EP_BULK                         (0x0000) /*!< BULK endpoint */
#define EP_CONTROL                      (0x0200) /*!< CONTROL endpoint */
#define EP_ISO                          (0x0400) /*!< ISOCHRONOUS endpoint */
#define EP_INTERRUPT                    (0x0600) /*!< INTERRUPT endpoint */
#define EP_CTL_MASK                     (~EP_CTL & EPCSR_MASK)

/**
  * @brief Endpoint kind control mask
  */
#define EPKCTL_MASK                     (~EP_KCTL & EPCSR_MASK)

/**
  * @brief TX_STA[1:0] status for Tx transfer
  */
#define EPTX_DISABLED                   (0x0000) /*!< Ignore all transmission request of this endpoint */
#define EPTX_STALL                      (0x0010) /*!< STALL handshake status */
#define EPTX_NAK                        (0x0020) /*!< NAK handshake status */
#define EPTX_VALID                      (0x0030) /*!< Enable this endpoint for transmission */
#define EPTX_DTGMASK                    (EPTX_STA | EPCSR_MASK)

/**
  * @brief RX_STA[1:0] status for Rx transfer
  */
#define EPRX_DISABLED                   (0x0000) /*!< Ignore all reception requests of this endpoint */
#define EPRX_STALL                      (0x1000) /*!< STALL handshake status */
#define EPRX_NAK                        (0x2000) /*!< NAK handshake status */
#define EPRX_VALID                      (0x3000) /*!< Enable endpoint for reception */
#define EPRX_DTGMASK                    (EPRX_STA | EPCSR_MASK)

/**
  * @brief Endpoint receive/transmission counter register bit definitions
  */
#define EPRXCNTR_BLKSIZ                 (0x8000)
#define EPRXCNTR_BLKNUM                 (0x7C00)
#define EPRXCNTR_CNT                    (0x03FF)

#define EPTXCNTR_CNT                    (0x03FF)

/**
  * @brief Endpoint receive/transmission counter register bit offset
  */
#define BLKSIZE_OFFSET                  (0x01)
#define BLKNUM_OFFSET                   (0x05)
#define RXCNT_OFFSET                    (0x0A)

#define TXCNT_OFFSET                    (0x0A)

#define BLKSIZE32_MASK                  (0x1f)
#define BLKSIZE2_MASK                   (0x01)

#define BLKSIZE32_OFFSETMASK            (0x05)
#define BLKSIZE2_OFFSETMASK             (0x01)

/**
  * @}
  */

/** @defgroup USB_Exported_Macros
  * @{
  */

/**@defgroup USB_Register_Macros
  *@{
  */

/* Set CTLR */
#define _SetCTLR(RegValue)              (*CTLR = (uint16_t)RegValue)

/* Get CTLR */
#define _GetCTLR()                      ((uint16_t) *CTLR)

/* Set IFR */
#define _SetIFR(RegValue)               (*IFR = (uint16_t)RegValue)

/* Get IFR */
#define _GetIFR()                       ((uint16_t) *IFR)

/* Set AR */
#define _SetAR(RegValue)                (*AR = (uint16_t)RegValue)

/* Get AR */
#define _GetAR()                        ((uint16_t) *AR)

/* Set BAR */
#define _SetBAR(RegValue)               (*BAR = (uint16_t)(RegValue & 0xFFF8))

/* Get BAR */
#define _GetBAR()                       ((uint16_t) *BAR)

/* Get SR */
#define _GetSR()                        ((uint16_t) *SR)

/* Set EPxCSR */
#define _SetEPxCSR(EpID,RegValue)       (*(EP0CSR + EpID) = (uint16_t)RegValue)

/* Get EPxCSR*/
#define _GetEPxCSR(EpID)                ((uint16_t)(*(EP0CSR + EpID)))

/* Set LPMCNTR */
#define _SetLPM_CNTR(RegValue)          (*LPM_CNTR = (uint16_t)RegValue)

/* Get LPMCNTR */
#define _GetLPM_CNTR()                  ((uint16_t) *LPM_CNTR)

/* Set LPMISTR */
#define _SetLPM_ISTR(RegValue)          (*LPM_ISTR = (uint16_t)RegValue)

/* Get LPMISTR */
#define _GetLPM_ISTR()                  ((uint16_t) *LPM_ISTR)

/* Set SUBEP0R */
#define _SetSUBEP0R(RegValue)           (*SUB_EP0REG = (uint16_t)RegValue)

/* Get SUBEP0R */
#define _GetSUBEP0R()                   ((uint16_t)*SUB_EP0REG)

/**
  * @}
  */

/** @defgroup USB_Operation_Macros
  * @{
  */

/**
  * @brief  Endpoint type setting and getting(bits EP_CTL[1:0] in endpoint control and status register)
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Type: endpoint type
  *   This parameter can be any one of the following values:
  *     @arg USB_EPTYPE_CONTROL: control transfer endpoint
  *     @arg USB_EPTYPE_BULK: bulk transfer endpoint
  *     @arg USB_EPTYPE_INT: interrupt transfer endpoint
  *     @arg USB_EPTYPE_ISOC: isochronous transfer endpoint
  * @retval None
  */
#define _SetEPType(EpID, Type)          (_SetEPxCSR(EpID, ((_GetEPxCSR(EpID) & EP_CTL_MASK) | Type)))

#define _GetEPType(EpID)                (_GetEPxCSR(EpID) & EP_CTL)


/**
  * @brief  Tx transfer status setting and getting (bits EPTX_STA[1:0])
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  State: new state
  *   This parameter can be any one of the following values:
  *     @arg EPTX_DISABLED 
  *     @arg EPTX_VALID
  *     @arg EPTX_NAK
  *     @arg EPTX_STALL
  * @retval None
  */
#define _SetEPTxStatus(EpID, State) {\
    register uint16_t _RegVal;       \
    _RegVal = _GetEPxCSR(EpID) & EPTX_DTGMASK;\
    _SetEPxCSR(EpID, (_RegVal ^ State));    \
}

#define _GetEPTxStatus(EpID)            ((uint16_t)_GetEPxCSR(EpID) & EPTX_STA)

/**
  * @brief  Rx transfer status setting and getting (bits EPRX_STA[1:0])
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  State: new state
  *   This parameter can be any one of the following values:
  *     @arg EPRX_DISABLED 
  *     @arg EPRX_VALID
  *     @arg EPRX_NAK
  *     @arg EPRX_STALL
  * @retval None
  */
#define _SetEPRxStatus(EpID, State) {\
    register uint16_t _RegVal;   \
    _RegVal = _GetEPxCSR(EpID) & EPRX_DTGMASK;\
    _SetEPxCSR(EpID, (_RegVal ^ State)); \
}

#define _GetEPRxStatus(EpID)            ((uint16_t)_GetEPxCSR(EpID) & EPRX_STA)

/**
  * @brief  Rx and Tx transfer status setting (bits EPRX_STA[1:0] & EPTX_STA[1:0])
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  StateRx: new Rx state
  *   This parameter can be any one of the following values:
  *     @arg EPRX_DISABLED 
  *     @arg EPRX_VALID
  *     @arg EPRX_NAK
  *     @arg EPRX_STALL
  * @param  StateTx: new Tx state
  *   This parameter can be any one of the following values:
  *     @arg EPTX_DISABLED 
  *     @arg EPTX_VALID
  *     @arg EPTX_NAK
  *     @arg EPTX_STALL
  * @retval None
  */
#define _SetEPRxTxStatus(EpID, StateRx, StateTx) {\
    register uint16_t _RegVal;   \
    _RegVal = _GetEPxCSR(EpID) & (EPRX_DTGMASK | EPTX_STA) ;\
    _SetEPxCSR(EpID, ((_RegVal ^ StateRx) ^ StateTx));    \
}

/**
  * @brief  Set and Clear endpoint kind (bit EP_KCTL).
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _SetEP_KIND(EpID)               (_SetEPxCSR(EpID, ((_GetEPxCSR(EpID) | EP_KCTL) & EPCSR_MASK)))

#define _ClearEP_KIND(EpID)             (_SetEPxCSR(EpID, (_GetEPxCSR(EpID) & EPKCTL_MASK)))

/**
  * @brief  Set and Clear directly STATUS_OUT state of endpoint
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _Set_Status_Out(EpID)           _SetEP_KIND(EpID)

#define _Clear_Status_Out(EpID)         _ClearEP_KIND(EpID)

/**
  * @brief  Set and Clear directly double buffered feature of endpoint
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _SetEPDoubleBuff(EpID)          _SetEP_KIND(EpID)

#define _ClearEPDoubleBuff(EpID)        _ClearEP_KIND(EpID)

/**
  * @brief  Clear bit EPRX_ST / EPTX_ST in the endpoint control and status register
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _ClearEPRX_ST(EpID)             (_SetEPxCSR(EpID, _GetEPxCSR(EpID) & 0x7FFF & EPCSR_MASK))

#define _ClearEPTX_ST(EpID)             (_SetEPxCSR(EpID, _GetEPxCSR(EpID) & 0xFF7F & EPCSR_MASK))

/**
  * @brief  Toggle and Clear EPRX_DTG bit in the endpoint control and status register
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _ToggleDTG_RX(EpID)             (_SetEPxCSR(EpID, EPRX_DTG | (_GetEPxCSR(EpID) & EPCSR_MASK)))

#define _ClearDTG_RX(EpID)              if((_GetEPxCSR(EpID) & EPRX_DTG) != 0) _ToggleDTG_RX(EpID)

/**
  * @brief  Toggle and Clear EPTX_DTG bit in the endpoint control and status register
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _ToggleDTG_TX(EpID)             (_SetEPxCSR(EpID, EPTX_DTG | (_GetEPxCSR(EpID) & EPCSR_MASK)))

#define _ClearDTG_TX(EpID)              if((_GetEPxCSR(EpID) & EPTX_DTG) != 0) _ToggleDTG_TX(EpID)

/**
  * @brief  Toggle SW_BUF bit in the double buffered endpoint
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _ToggleSWBUF_TX(EpID)           _ToggleDTG_RX(EpID)

#define _ToggleSWBUF_RX(EpID)           _ToggleDTG_TX(EpID)

/**
  * @brief  Set and Get endpoint address
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Addr: endpoint address
  * @retval None
  */
#define _SetEPAddress(EpID,Addr)        _SetEPxCSR(EpID, (_GetEPxCSR(EpID) & EPCSR_MASK) | Addr)

#define _GetEPAddress(EpID)             ((uint8_t)(_GetEPxCSR(EpID) & EP_AR))

/**
  * @brief  Get endpoint transmission buffer address
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _GetEPTXARn(EpID)               ((uint32_t *)((_GetBAR() + EpID * 8) * 2 + PBA_ADDR))

/**
  * @brief  Get endpoint transmission byte count
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _GetEPTXCNTx(EpID)              ((uint32_t *)((_GetBAR() + EpID * 8 + 2) * 2 + PBA_ADDR))

/**
  * @brief  Get endpoint reception buffer address
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _GetEPRXARn(EpID)               ((uint32_t *)((_GetBAR() + EpID * 8 + 4) * 2 + PBA_ADDR))

/**
  * @brief  Get endpoint reception byte count
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _GetEPRXCNTx(EpID)              ((uint32_t *)((_GetBAR() + EpID * 8 + 6) * 2 + PBA_ADDR))

/**
  * @brief  Set Tx/Rx buffer address
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Addr: address to be set (must be word aligned)
  * @retval None
  */
#define _SetEPTxAddr(EpID, Addr)        (*_GetEPTXARn(EpID) = (Addr & ~((uint16_t)1)))
#define _SetEPRxAddr(EpID, Addr)        (*_GetEPRXARn(EpID) = (Addr & ~((uint16_t)1)))

/**
  * @brief  Get Tx/Rx buffer address.
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval Address of the buffer
  */
#define _GetEPTxAddr(EpID)              ((uint16_t)*_GetEPTXARn(EpID))
#define _GetEPRxAddr(EpID)              ((uint16_t)*_GetEPRXARn(EpID))

/**
  * @brief  Set the reception buffer byte count register when 1 block is 32 bytes
  * @param  pdwReg: reception buffer byte count register
  * @param  Count: byte count value
  * @retval None
  */
#define _BlocksOf32(pdwReg, Count) {\
    register uint16_t BlockNum = Count >> BLKSIZE32_OFFSETMASK;\
    if((Count & BLKSIZE32_MASK) == 0)\
      BlockNum--;\
    *pdwReg = (uint32_t)((BlockNum << RXCNT_OFFSET) | EPRXCNTR_BLKSIZ);\
}

/**
  * @brief  Set the reception buffer byte count register when 1 block is 2 bytes
  * @param  pdwReg: reception buffer byte count register
  * @param  Count: byte count value
  * @retval None
  */
#define _BlocksOf2(pdwReg, Count) {\
    register uint16_t BlockNum = Count >> BLKSIZE2_OFFSETMASK;\
    if((Count & BLKSIZE2_MASK) != 0)\
      BlockNum++;\
    *pdwReg = (uint32_t)(BlockNum << RXCNT_OFFSET);\
}

/**
  * @brief  Set buffer0 reception byte count when use double buffer
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Count: byte count value
  * @retval None
  */
#define _SetEPRxDblBuf0Count(EpID, Count) {\
    uint32_t *pdwReg = _GetEPTXCNTx(EpID); \
    if(Count > 62) { _BlocksOf32(pdwReg, Count); } \
    else { _BlocksOf32(pdwReg, Count); } \
}

/**
  * @brief  Set Tx/Rx buffer byte count
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Count: byte count value
  * @retval None
  */
#define _SetEPTxCount(EpID, Count)      (*_GetEPTXCNTx(EpID) = Count)
#define _SetEPRxCount(EpID, Count) {\
    uint32_t *pdwReg = _GetEPRXCNTx(EpID); \
    if(Count > 62) { _BlocksOf32(pdwReg, Count); } \
    else { _BlocksOf2(pdwReg, Count); } \
}

/**
  * @brief  Get Tx/Rx buffer byte count
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval Byte count value
  */
#define _GetEPTxCount(EpID)((uint16_t)  (*_GetEPTXCNTx(EpID)) & EPTXCNTR_CNT)
#define _GetEPRxCount(EpID)((uint16_t)  (*_GetEPRXCNTx(EpID)) & EPRXCNTR_CNT)

/**
  * @brief  Sets buffer 0/1 address when use double buffer
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Buf0Addr: buffer0 address
  * @param  Buf1Addr: buffer1 address
  * @retval None
  */
#define _SetEPDblBuf0Addr(EpID, Buf0Addr) {_SetEPTxAddr(EpID, Buf0Addr);}
#define _SetEPDblBuf1Addr(EpID, Buf1Addr) {_SetEPRxAddr(EpID, Buf1Addr);}

/**
  * @brief  Sets a double buffer endpoint addresses
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Buf0Addr: buffer0 address
  * @param  Buf1Addr: buffer1 address
  * @retval None
  */
#define _SetEPDblBufAddr(EpID, Buf0Addr, Buf1Addr) { \
    _SetEPDblBuf0Addr(EpID, Buf0Addr);\
    _SetEPDblBuf1Addr(EpID, Buf1Addr);\
}

/**
  * @brief  Get a double buffer endpoint buffer 0/1 address
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval None
  */
#define _GetEPDblBuf0Addr(EpID)         (_GetEPTxAddr(EpID))
#define _GetEPDblBuf1Addr(EpID)         (_GetEPRxAddr(EpID))

/**
  * @brief  Set buffer 0/1 byte count register in a double buffer endpoint
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Dir: endpoint direction
  *   This parameter can be any one of the following values:
  *     @arg DBUF_EP_OUT = OUT
  *     @arg DBUF_EP_IN  = IN
  * @param  Count: byte count value
  * @retval None
  */
#define _SetEPDblBuf0Count(EpID, Dir, Count)  { \
    if(Dir == DBUF_EP_OUT)\
    {_SetEPRxDblBuf0Count(EpID, Count);} \
    else if(Dir == DBUF_EP_IN)\
    {*_GetEPTXCNTx(EpID) = (uint32_t)Count;} \
}

#define _SetEPDblBuf1Count(EpID, Dir, Count)  { \
    if(Dir == DBUF_EP_OUT)\
    {_SetEPRxCount(EpID, Count);}\
    else if(Dir == DBUF_EP_IN)\
    {*_GetEPRXCNTx(EpID) = (uint32_t)Count;} \
}

#define _SetEPDblBuffCount(EpID, Dir, Count) {\
    _SetEPDblBuf0Count(EpID, Dir, Count); \
    _SetEPDblBuf1Count(EpID, Dir, Count); \
}

/**
  * @brief  Get buffer 0/1 byte count
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval buffer byte count
  */
#define _GetEPDblBuf0Count(EpID)        (_GetEPTxCount(EpID))
#define _GetEPDblBuf1Count(EpID)        (_GetEPRxCount(EpID))

/**
  * @brief   Set double buffer endpoint status to STALL
  * @param   EpID: endpoint identifier which is in (0..7)
  * @param   Dir: endpoint direction
  *   This parameter can be any one of the following values:
  *     @arg DBUF_EP_OUT: OUT direction
  *     @arg DBUF_EP_IN: IN direction
  * @retval  None
  */
#define _SetDouBleBuffEPStall(EpID, Dir) {\
    if (Dir == DBUF_EP_OUT) \
    { \
        _SetEPxCSR(EpID, _GetEPxCSR(EpID) & ~EPRX_STALL); \
    } \
    else if (Dir == DBUF_EP_IN) \
    { \
        _SetEPxCSR(EpID, _GetEPxCSR(EpID) & ~EPTX_STALL); \
    } \
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* __GD32F1X0_USB_REGS_H */

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
