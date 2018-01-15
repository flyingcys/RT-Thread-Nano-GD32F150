/**
  ******************************************************************************
  * @file    gd32_it.h 
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Interrupt handlers header files
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_IT_H
#define __GD32F1X0_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void NMI_Handler          (void);
void HardFault_Handler    (void);
void SVC_Handler          (void);
void PendSV_Handler       (void);
void SysTick_Handler      (void);
void USB_FS_LP_IRQHandler (void);
void EXTI4_15_IRQHandler  (void);

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_IT_H */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
