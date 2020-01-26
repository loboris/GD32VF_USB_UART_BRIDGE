/*!
    \file  main.c
    \brief USB CDC ACM device

    \version 2019-6-5, V1.0.0, demo for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "gd32v_pjt_include.h"

#define PROG_ID (u8 *)"USB UART BRIDGE v0.6"

usb_core_driver USB_OTG_dev = 
{
    .dev = {
        .desc = {
            .dev_desc       = (uint8_t *)&device_descriptor,
            .config_desc    = (uint8_t *)&configuration_descriptor,
            .strings        = usbd_strings,
        }
    }
};

static uint8_t rxbuffer[UART_RXBUFFER_SIZE];
static __IO uint16_t rxcount = 0; 
static uint8_t cdcbuffer[UART_RXBUFFER_SIZE];
static uint8_t rcvstr[CDC_ACM_DATA_PACKET_SIZE] = {0};
static uint8_t txbuffer[CDC_ACM_DATA_PACKET_SIZE+8] = {'^'};
static __IO uint16_t txcount = 0; 
static __IO uint16_t txbuffer_idx = 0; 

static uint8_t uart_sending = 0;


//---------------------------
static void init_clocks(void)
{	
	// enable GPIO clocks
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    // enable USART clocks
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_USART2);
}

//------------------------------
static void init_uart_pins(void)
{
    gpio_init(UART0_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, UART0_TXPIN);
    gpio_init(UART0_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, UART0_RXPIN);
    gpio_init(UART0_PORT, UART0_RTS_DTR_MODE, GPIO_OSPEED_50MHZ, UART0_RTSPIN);
    gpio_init(UART0_PORT, UART0_RTS_DTR_MODE, GPIO_OSPEED_50MHZ, UART0_DTRPIN);
    gpio_bit_set(UART0_PORT, UART0_TXPIN);
    gpio_bit_set(UART0_PORT, UART0_RTSPIN);
    gpio_bit_set(UART0_PORT, UART0_DTRPIN);

    gpio_init(UART2_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, UART2_TXPIN);
    gpio_init(UART2_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, UART2_RXPIN);
    gpio_init(UART2_PORT, UART2_RTS_DTR_MODE, GPIO_OSPEED_50MHZ, UART2_RTSPIN);
    gpio_init(UART2_PORT, UART2_RTS_DTR_MODE, GPIO_OSPEED_50MHZ, UART2_DTRPIN);
    gpio_bit_set(UART2_PORT, UART2_TXPIN);
    gpio_bit_set(UART2_PORT, UART2_RTSPIN);
    gpio_bit_set(UART2_PORT, UART2_DTRPIN);
}

//----------------------------------
static void init_uart(uint32_t uart)
{
    if ((uart != USART0) && (uart != USART2)) return;

    // Reset uart
    usart_deinit(uart);

	// Configure uart
    usart_baudrate_set(uart, linecoding.dwDTERate);
    usart_word_length_set(uart, linecoding.bDataBits);
    usart_stop_bit_set(uart, linecoding.bCharFormat);
    usart_parity_config(uart, linecoding.bParityType);
    usart_hardware_flow_rts_config(uart, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(uart, USART_CTS_DISABLE);
    usart_receive_config(uart, USART_RECEIVE_ENABLE);
    usart_transmit_config(uart, USART_TRANSMIT_ENABLE);

    rxcount = 0;
    txcount = 0;
    txbuffer_idx = 0;
    uart_sending = 0;
    uart_send_count = 0;
    uart_receive_count = 0;
    uart_error_count = 0;

    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    if (uart == USART0) eclic_irq_enable(USART0_IRQn, 1, 3);
    else eclic_irq_enable(USART2_IRQn, 1, 3);

    usart_interrupt_flag_clear(uart, USART_INT_FLAG_RBNE);
    usart_interrupt_enable(uart, USART_INT_RBNE);
    usart_interrupt_disable(uart, USART_INT_TBE);
    usart_interrupt_disable(uart, USART_INT_TC);

    usart_enable(uart);

    #if USE_LEDS
    LEDG((uart == USART0));
    #endif
    active_uart = uart;
}

//------------------------------------
static void deinit_uart(uint32_t uart)
{	
    if ((uart != USART0) && (uart != USART2)) return;

    // Reset uart
    usart_deinit(uart);
    /*
    usart_interrupt_disable(uart, USART_INT_RBNE);
    if (uart == USART0) eclic_irq_disable(USART0_IRQn);
    else eclic_irq_disable(USART2_IRQn);
    usart_receive_config(uart, USART_RECEIVE_DISABLE);
    usart_transmit_config(uart, USART_TRANSMIT_DISABLE);
    usart_interrupt_disable(uart, USART_INT_RBNE);
    usart_interrupt_disable(uart, USART_INT_TBE);
    usart_interrupt_disable(uart, USART_INT_TC);
    */
}

//-------------------------
void _uart_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(active_uart, USART_INT_FLAG_RBNE)) {
        // * Read data buffer not empty, copy received byte to rx buffer
        uint8_t s = (uint8_t)(USART_STAT(active_uart) & (USART_STAT_PERR | USART_STAT_FERR | USART_STAT_NERR | USART_STAT_ORERR));
        uint8_t b = usart_data_receive(active_uart);
        if (s == 0) {
            if (rxcount < UART_RXBUFFER_SIZE) {
                rxbuffer[rxcount++] = b;
                uart_receive_count++;
            }
            else uart_error_count++;
        }
        else uart_error_count++;
    }
    if (RESET != usart_interrupt_flag_get(active_uart, USART_INT_FLAG_TBE)) {
        // * Transmit data buffer empty, send next byte from tx buffer
        if (txbuffer_idx >= txcount) {
            // last byte transmitted
            usart_interrupt_disable(active_uart, USART_INT_TBE);
            uart_sending = 0;
        }
        else {
            usart_data_transmit(active_uart, txbuffer[txbuffer_idx++]);
        }
    }
}

//==========================
void USART0_IRQHandler(void)
{
    _uart_IRQHandler();
}

//==========================
void USART2_IRQHandler(void)
{
    _uart_IRQHandler();
}

// Send USB CDC data from buffer to UART
//--------------------------------------
static void uart_send_data(uint32_t len)
{
    uart_send_count += len;

    txcount = len;
    txbuffer_idx = 0;
    uart_sending = 1;
    usart_interrupt_enable(active_uart, USART_INT_TBE);
    // send 1st byte
    //usart_data_transmit(active_uart, txbuffer[txbuffer_idx++]);
}

//------------------------------
static void uart_wait_sent(void)
{
    if (uart_sending) {
        // wait until previous data are sent
        while (uart_sending) {
            #if USE_LEDS
            LEDR(0);
            #endif
        }
        while (RESET == usart_flag_get(active_uart, USART_FLAG_TC)) {
        }
        #if USE_LEDS
        LEDR(1);
        #endif
    }
}

//===========================
void delay_ms(uint32_t count)
{
	uint64_t start_mtime, delta_mtime;

	// Don't start measuruing until we see an mtime tick
	uint64_t tmp = get_timer_value();
	do {
	    start_mtime = get_timer_value();
	} while (start_mtime == tmp);

	do {
	    delta_mtime = get_timer_value() - start_mtime;
	} while(delta_mtime <(SystemCoreClock/4000.0 *count ));
}

//=====================
uint64_t get_time(void)
{
	uint64_t curr_mtime;

	// Wait for next mtime tick
	uint64_t tmp = get_timer_value();
	do {
	    curr_mtime = get_timer_value();
	} while (curr_mtime == tmp);
    return curr_mtime; // time in system ticks
}

//============
int main(void)
{
    // system clocks configuration
    init_clocks();

    // Disable JTAG
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, ENABLE);

    #if USE_LEDS
    // Initialize LEDs
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    LEDR(0);
    LEDG(1);
    #endif

    #if USE_DISPLAY
    // Initialize Display
    Lcd_Init();
    LCD_Clear(BLACK);
    BACK_COLOR=DGRAY;
    LCD_ShowStr(0, 0, PROG_ID, CYAN, OPAQUE);
    BACK_COLOR=BLACK;
    LCD_ShowStr(0, 64, (u8 *)("USB NOT ENUMERATED!\r"), YELLOW, OPAQUE);
    #endif

    // Enable the global interrupt
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL2_PRIO2);

    // Initialize UART
    init_uart_pins();
    init_uart(USART0);

    // Initialize USB
    usb_rcu_config();
    usb_timer_init();
    usb_intr_config();
    usbd_init (&USB_OTG_dev, USB_CORE_ENUM_FS, &usbd_cdc_cb);

    // wait until USB device is enumerated successfully
    while (USBD_CONFIGURED != USB_OTG_dev.dev.cur_status) {
    }
    #if USE_DISPLAY
    LCD_ShowStr(0, 64, (u8 *)("Ready.\r"), YELLOW, OPAQUE);
    #endif
    #if USE_LEDS
    LEDR(1);
    #endif

    while (1) {
        if (USBD_CONFIGURED == USB_OTG_dev.dev.cur_status) {
            if (uart_change_req > 0) {
                //uart_wait_sent();
                // UART change requested
                if ((uart_change_req == 1) && (active_uart != USART0)) {
                    // activate UART0
                    deinit_uart(USART2);
                    init_uart(USART0);
                }
                else if ((uart_change_req == 2) && (active_uart != USART2)) {
                    // activate UART2
                    if (active_uart == USART0) deinit_uart(USART0);
                    init_uart(USART2);
                }
                uart_change_req = 0;
                lcd_showSettings();
            }

            // === USB CDC -> UART ===
            if (packet_receive) {
                if (receive_length > 0) {
                    uart_wait_sent();
                    memcpy(txbuffer, rcvstr, receive_length);
                    uart_send_data(receive_length);
                }
                packet_receive = 0;
                // receive more data from host
                usbd_ep_recev(&USB_OTG_dev, CDC_ACM_DATA_OUT_EP, rcvstr, CDC_ACM_DATA_PACKET_SIZE);
            }

            // === UART -> USB CDC ===
            if (packet_sent) {
                // free to send new USB packet
                usart_interrupt_disable(active_uart, USART_INT_RBNE);
                if (rxcount > 0) {
                    // we have some data waiting in UART buffer
                    uint16_t inbuffer = rxcount;
                    rxcount = 0;
                    memcpy(cdcbuffer, rxbuffer, inbuffer);
                    usart_interrupt_enable(active_uart, USART_INT_RBNE);
                    packet_sent = 0;
                    usbd_ep_send(&USB_OTG_dev, CDC_ACM_DATA_IN_EP, cdcbuffer, inbuffer);
                    send_count += inbuffer;
                }
                else usart_interrupt_enable(active_uart, USART_INT_RBNE);
            }
        }
        else {
            #if USE_DISPLAY
            LCD_ShowStr(0, 64, (u8 *)("USB NOT ENUMERATED!\r"), YELLOW, OPAQUE);
            #endif
            #if USE_LEDS
            LEDG(1);
            LEDR(0);
            #endif

            // wait until USB device is enumerated again
            while (USBD_CONFIGURED != USB_OTG_dev.dev.cur_status) {
            }

            #if USE_DISPLAY
            LCD_ShowStr(0, 64, (u8 *)("USB ENUMERATED!\r"), GREEN, OPAQUE);
            #endif
            #if USE_LEDS
            LEDR(1);
            #endif
        }
    }
}

