/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-08-30     Aubr.Cool       the first version
 */
#include <rtthread.h>

#ifdef RT_USING_UART
#include "gd32f1x0.h"

#include <rtdevice.h>
#define UART_RX_BUFSZ 8

struct gd32_uart
{
	struct rt_device parent;
	struct rt_ringbuffer rx_rb;

	USART_TypeDef* uart_base;
	IRQn_Type uart_irq;
	
	rt_uint8_t rx_buffer[UART_RX_BUFSZ];
};

#ifdef RT_USING_UART1
struct gd32_uart uart1_device;

static gd32_uart_recv_handler(struct gd32_uart *uart)
{
    if(USART_GetIntBitState(uart->uart_base, USART_INT_RBNE) != RESET)
    {
        rt_ringbuffer_putchar_force(&(uart->rx_rb), (rt_uint8_t)USART_DataReceive(uart->uart_base));
        if(uart->parent.rx_indicate != RT_NULL)
        {
            uart->parent.rx_indicate(&uart->parent, rt_ringbuffer_data_len(&uart->rx_rb));
        }
    }
}

void USART1_IRQHandler(void)
{
    rt_interrupt_enter();

    gd32_uart_recv_handler(&uart1_device);

    rt_interrupt_leave();
}
#endif

static void gd32_uart_init(USART_TypeDef *uart_base)
{
    if(uart_base == USART1)
    {
        USART_InitPara USART_InitStruct;
        GPIO_InitPara GPIO_InitStructure;

        RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA, ENABLE);
        RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_USART1, ENABLE); 

        GPIO_DeInit( GPIOA );
        GPIO_PinAFConfig(GPIOA, GPIO_PINSOURCE9, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PINSOURCE10, GPIO_AF_1);

        GPIO_InitStructure.GPIO_Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_10MHZ;
        GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        USART_DeInit(USART1);
        USART_InitStruct.USART_BRR = 115200;
        USART_InitStruct.USART_WL = USART_WL_8B;
        USART_InitStruct.USART_STBits = USART_STBITS_1;
        USART_InitStruct.USART_Parity = USART_PARITY_RESET;
        USART_InitStruct.USART_RxorTx = USART_RXORTX_RX | USART_RXORTX_TX;
        USART_InitStruct.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
        USART_Init(USART1, &USART_InitStruct);

        USART_Enable(USART1, ENABLE);        

		USART_INT_Set(USART1, USART_INT_RBNE, ENABLE);
		USART_INT_Set(USART1, USART_INT_TBE, DISABLE);
    }
    else
    {
        RT_ASSERT(uart_base == USART1);
    }
}

static void gd32_uart_nvic_init(IRQn_Type uart_irq)
{
    NVIC_InitPara  NVIC_InitStructure;

    NVIC_PRIGroup_Enable(NVIC_PRIGROUP_0);

    NVIC_InitStructure.NVIC_IRQ = uart_irq;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static rt_err_t rt_uart_init(rt_device_t dev)
{
    struct gd32_uart *uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct gd32_uart *)dev;

    gd32_uart_init(uart->uart_base);
    gd32_uart_nvic_init(uart->uart_irq);

    return RT_EOK;
}

static rt_err_t rt_uart_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct gd32_uart *uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct gd32_uart *)dev;

    if(dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        NVIC_EnableIRQ(uart->uart_irq);
    }
    
    return RT_EOK;
}

static rt_err_t rt_uart_close(rt_device_t dev)
{
    struct gd32_uart *uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct gd32_uart *)dev;

    if(dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        NVIC_DisableIRQ(uart->uart_irq);
    }
    return RT_EOK;
}

static rt_size_t rt_uart_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_size_t length;
    rt_base_t level;
    struct gd32_uart *uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct gd32_uart *)dev;

    level = rt_hw_interrupt_disable();
    length = rt_ringbuffer_get(&uart->rx_rb, buffer, size);
    rt_hw_interrupt_enable(level);
    
    return length;
}

static rt_size_t rt_uart_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    char *ptr = (char *)buffer;
    struct gd32_uart *uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct gd32_uart *)dev;

    if(dev->open_flag & RT_DEVICE_FLAG_STREAM)
    {
        while(size)
        {
            if(*ptr == '\n')
            {
                while (USART_GetBitState(USART1, USART_FLAG_TC) == RESET);
                USART_DataSend(uart->uart_base, '\r');
            }
            //
            while (USART_GetBitState(USART1, USART_FLAG_TC) == RESET);
            USART_DataSend(uart->uart_base, *ptr);

            ptr ++;
            size --;
        }
    }
    else
    {
        while(size)
        {
            while (USART_GetBitState(USART1, USART_FLAG_TC) == RESET);
            USART_DataSend(uart->uart_base, *ptr);
            
            ptr ++;
            size --;
        }

    }
    return RT_EOK;
}

int rt_hw_usart_init(void)
{
    struct gd32_uart *uart;
	
#ifdef RT_USING_UART1
	uart = &uart1_device;
	
	uart->parent.type = RT_Device_Class_Char;
	uart->uart_base = USART1;
	uart->uart_irq = USART1_IRQn;

	rt_ringbuffer_init(&uart->rx_rb, uart->rx_buffer, sizeof(uart->rx_buffer));

	uart->parent.init = rt_uart_init;
	uart->parent.open = rt_uart_open;
	uart->parent.close = rt_uart_close;
	uart->parent.read = rt_uart_read;
	uart->parent.write = rt_uart_write;
	uart->parent.control = RT_NULL;
	uart->parent.user_data = RT_NULL;

	rt_device_register(&uart->parent, "uart1", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
#endif

	return 0;
}
INIT_BOARD_EXPORT(rt_hw_usart_init);
#endif
