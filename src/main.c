/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// RPI Pico SDK includes
#include <pico/multicore.h>
#include <bsp/board.h>
#include <hardware/structs/sio.h>

#include <hardware/pio.h>
#include <uart_tx.pio.h>
#include <uart_rx.pio.h>

// Tiny USB includes
#include <tusb.h>

// App includes
#include <app_uart.h>

/* Struct holding details on each of the hardware UART peripherals */
static const hard_uart_id_t HARD_UART_ID[NUM_HARD_UARTS] = {
    {
        .uid = UART0_ID_NUM,
        .hard_inst = uart0,
        .hard_tx_pin = UART0_TX_PIN,
        .hard_rx_pin = UART0_RX_PIN,
    },
    {
        .uid = UART1_ID_NUM,
        .hard_inst = uart1,
        .hard_tx_pin = UART1_TX_PIN,
        .hard_rx_pin = UART1_RX_PIN,
    }};

/* Struct holding details on each of the PIO UART peripherals */
static const pio_uart_id_t PIO_UART_ID[NUM_PIO_UARTS] = {
    {
        .uid = UART2_ID_NUM,
        .pio_sm_num = PIO_SM0_NUM,
        .pio_sm_tx_pin = UART2_TX_PIN,
        .pio_sm_rx_pin = UART2_RX_PIN,
    },
    {
        .uid = UART3_ID_NUM,
        .pio_sm_num = PIO_SM1_NUM,
        .pio_sm_tx_pin = UART3_TX_PIN,
        .pio_sm_rx_pin = UART3_RX_PIN,
    }};

static uart_uid uart_ids[NUM_APP_UARTS] = {
    UART0_ID_NUM,
    UART1_ID_NUM,
    UART2_ID_NUM,
    UART3_ID_NUM};

static uart_data_t UART_DATA[CFG_TUD_CDC];

static PIO pio_uart_rx = pio0;
static PIO pio_uart_tx = pio1;

static uint32_t rx_pio_offset;
static uint32_t tx_pio_offset;

// Databits tiny usb cdc to rpi pico mapping
static inline uint8_t databits_usb2uart(uint8_t data_bits)
{
  switch (data_bits)
  {
  case USB_CDC_LC_FIVE_DBITS:
    return UART_FIVE_DBITS;
  case USB_CDC_LC_SIX_DBITS:
    return UART_SIX_DBITS;
  case USB_CDC_LC_SEVEN_DBITS:
    return UART_SEVEN_DBITS;
  case USB_CDC_LC_EIGHT_DBITS:
    return UART_EIGHT_DBITS;
  default:
    return UART_EIGHT_DBITS;
  }
}

// Parity tiny usb cdc to rpi pico mapping
static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
  switch (usb_parity)
  {
  case USB_CDC_LC_NO_PARITY:
    return UART_PARITY_NONE;
  case USB_CDC_LC_ODD_PARITY:
    return UART_PARITY_ODD;
  case USB_CDC_LC_EVEN_PARITY:
    return UART_PARITY_EVEN;
  default:
    return UART_PARITY_NONE;
  }
}

// Stopbits tiny usb cdc to rpi pico mapping
static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
  switch (stop_bits)
  {
  case USB_CDC_LC_STOP_BIT_1:
    return UART_STOP_BIT_1;
  case UART_STOP_BIT_2:
    return UART_STOP_BIT_2;
  default:
    return UART_STOP_BIT_1;
  }
}

// Update the configs of the uart devices. Currently does nothing if PIO.
void update_uart_cfg(uint8_t itf)
{
  switch (itf)
  {
  case UART0_ID_NUM:
  case UART1_ID_NUM:
    /* code */
    const hard_uart_id_t *hard_ui = &HARD_UART_ID[itf];
    uart_data_t *ud = &UART_DATA[itf];

    mutex_enter_blocking(&ud->lc_mtx);

    if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate)
    {
      uart_set_baudrate(hard_ui->hard_inst, ud->usb_lc.bit_rate);
      ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
    }

    if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
        (ud->usb_lc.parity != ud->uart_lc.parity) ||
        (ud->usb_lc.data_bits != ud->uart_lc.data_bits))
    {
      uart_set_format(hard_ui->hard_inst,
                      databits_usb2uart(ud->usb_lc.data_bits),
                      stopbits_usb2uart(ud->usb_lc.stop_bits),
                      parity_usb2uart(ud->usb_lc.parity));
      ud->uart_lc.data_bits = ud->usb_lc.data_bits;
      ud->uart_lc.parity = ud->usb_lc.parity;
      ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
    }

    mutex_exit(&ud->lc_mtx);
    break;

  default:
    break;
  }
}

void usb_read_bytes(uint8_t itf)
{
  uint32_t len = tud_cdc_n_available(itf);

  if (len)
  {
    uart_data_t *ud = &UART_DATA[itf];

    mutex_enter_blocking(&ud->usb_mtx);

    len = MIN(len, BUFFER_SIZE - ud->usb_pos);
    if (len)
    {
      uint32_t count;

      count = tud_cdc_n_read(itf, &ud->usb_buffer[ud->usb_pos], len);
      ud->usb_pos += count;
    }

    mutex_exit(&ud->usb_mtx);
  }
}

void usb_write_bytes(uint8_t itf)
{
  uart_data_t *ud = &UART_DATA[itf];

  if (ud->uart_pos)
  {
    uint32_t count;

    mutex_enter_blocking(&ud->uart_mtx);

    count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
    if (count < ud->uart_pos)
      memcpy(ud->uart_buffer, &ud->uart_buffer[count],
             ud->uart_pos - count);
    ud->uart_pos -= count;

    mutex_exit(&ud->uart_mtx);

    if (count)
      tud_cdc_n_write_flush(itf);
  }
}

void usb_cdc_process(uint8_t itf)
{
  uart_data_t *ud = &UART_DATA[itf];

  mutex_enter_blocking(&ud->lc_mtx);
  tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
  mutex_exit(&ud->lc_mtx);

  usb_read_bytes(itf);
  usb_write_bytes(itf);
}

void uart_read_bytes(uint8_t itf)
{
  uart_data_t *ud = &UART_DATA[itf];

  switch (itf)
  {
  case UART0_ID_NUM:
  case UART1_ID_NUM:
    const hard_uart_id_t *hard_ui = &HARD_UART_ID[itf];

    if (uart_is_readable(hard_ui->hard_inst))
    {

      mutex_enter_blocking(&ud->uart_mtx);

      while (uart_is_readable(hard_ui->hard_inst) &&
             ud->uart_pos < BUFFER_SIZE)
      {
        ud->uart_buffer[ud->uart_pos] = uart_getc(hard_ui->hard_inst);
        ud->uart_pos++;
      }
      mutex_exit(&ud->uart_mtx);
    }
    break;
  case UART2_ID_NUM:
  case UART3_ID_NUM:
    const pio_uart_id_t *pio_ui = &PIO_UART_ID[itf - CFG_TUD_CDC + NUM_PIO_UARTS];

    mutex_enter_blocking(&ud->uart_mtx);
    ud->uart_pos = uart_rx_program_read(pio_uart_rx, pio_ui->pio_sm_num, ud->uart_buffer);
    mutex_exit(&ud->uart_mtx);
    break;

  default:
    break;
  }
}

void uart_write_bytes(uint8_t itf)
{
  uart_data_t *ud = &UART_DATA[itf];

  switch (itf)
  {
  case UART0_ID_NUM:
  case UART1_ID_NUM:

    if (ud->usb_pos)
    {
      const hard_uart_id_t *hard_ui = &HARD_UART_ID[itf];

      mutex_enter_blocking(&ud->usb_mtx);

      uart_write_blocking(hard_ui->hard_inst, ud->usb_buffer, ud->usb_pos);
      ud->usb_pos = 0;

      mutex_exit(&ud->usb_mtx);
    }
    break;

  case UART2_ID_NUM:
  case UART3_ID_NUM:
    const pio_uart_id_t *pio_ui = &PIO_UART_ID[itf - CFG_TUD_CDC + NUM_PIO_UARTS];
    mutex_enter_blocking(&ud->usb_mtx);
    if (ud->usb_pos)
    {
      uart_tx_program_write(pio_uart_tx, pio_ui->pio_sm_num, (char *)ud->usb_buffer, (ud->usb_pos) + 1);
    }
    ud->usb_pos = 0;
    mutex_exit(&ud->usb_mtx);
    break;
  }
}

// Initialise the configs and data of the uart devices. Currently does nothing if PIO.
void init_uart_data(uint8_t itf)
{

  uart_data_t *ud = &UART_DATA[itf];

  /* USB CDC LC */
  ud->usb_lc.bit_rate = DEFAULT_BAUD_RATE;
  ud->usb_lc.data_bits = DEFAULT_USB_CDC_NUM_DATA_BITS;
  ud->usb_lc.parity = DEFAULT_USB_CDC_PARITY_MODE;
  ud->usb_lc.stop_bits = DEFAULT_USB_CDC_NUM_STOP_BITS;

  /* UART LC */
  ud->uart_lc.bit_rate = DEFAULT_BAUD_RATE;
  ud->uart_lc.data_bits = DEFAULT_UART_NUM_DATA_BITS;
  ud->uart_lc.parity = DEFAULT_UART_PARITY_MODE;
  ud->uart_lc.stop_bits = DEFAULT_UART_NUM_STOP_BITS;

  /* Buffer */
  ud->uart_pos = 0u;
  ud->usb_pos = 0u;

  /* Mutex */
  mutex_init(&ud->lc_mtx);
  mutex_init(&ud->uart_mtx);
  mutex_init(&ud->usb_mtx);

  switch (itf)
  {
  case UART0_ID_NUM:
  case UART1_ID_NUM:
    const hard_uart_id_t *hard_ui = &HARD_UART_ID[itf];
    /* Pinmux */
    gpio_set_function(hard_ui->hard_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(hard_ui->hard_rx_pin, GPIO_FUNC_UART);
    /* UART start */
    uart_init(hard_ui->hard_inst, ud->usb_lc.bit_rate);
    uart_set_hw_flow(hard_ui->hard_inst, false, false);
    uart_set_format(hard_ui->hard_inst, databits_usb2uart(ud->usb_lc.data_bits),
                    stopbits_usb2uart(ud->usb_lc.stop_bits),
                    parity_usb2uart(ud->usb_lc.parity));
    break;
  case UART2_ID_NUM:
  case UART3_ID_NUM:
    const pio_uart_id_t *pio_ui = &PIO_UART_ID[itf - CFG_TUD_CDC + NUM_PIO_UARTS];
    uart_rx_program_init(pio_uart_rx, pio_ui->pio_sm_num, rx_pio_offset, pio_ui->pio_sm_rx_pin, DEFAULT_BAUD_RATE);
    uart_tx_program_init(pio_uart_tx, pio_ui->pio_sm_num, tx_pio_offset, pio_ui->pio_sm_tx_pin, DEFAULT_BAUD_RATE);
    break;
  default:
    break;
  }
}

void core_0_app_init(void)
{
  board_init();
  rx_pio_offset = pio_add_program(pio_uart_rx, &uart_rx_program);
  tx_pio_offset = pio_add_program(pio_uart_tx, &uart_tx_program);

  uint8_t itf;
  for (itf = 0; itf < CFG_TUD_CDC; itf++)
    init_uart_data(itf);

  gpio_init(LED_PIN_NUMBER);
  gpio_set_dir(LED_PIN_NUMBER, GPIO_OUT);
}

void core_1_app_init(void)
{
  // Initialise USB stack
  tusb_init();
}

void core_0_app_task(void)
{
  while (1)
  {

    for (uint8_t itf = 0u; itf < CFG_TUD_CDC; itf++)
    {
      update_uart_cfg(itf);
      uart_read_bytes(itf);
      uart_write_bytes(itf);
    }
  }
}

void core_1_app_task(void)
{
  while (1)
  {
    uint8_t itf;
    bool con = false;

    tud_task();

    for (itf = 0; itf < CFG_TUD_CDC; itf++)
    {
      // Only process USB if connected to host serial
      if (tud_cdc_n_connected(itf))
      {
        con = true;
        usb_cdc_process(itf);
      }
    }

    // Turn on the LED if any of the CDC devices are connected to the host
    gpio_put(LED_PIN_NUMBER, con);
  }
}

/*------------- MAIN -------------*/

void core1_main(void)
{

  core_1_app_init();

  core_1_app_task();
}

int main(void)
{

  // Must be called before core 1 has started
  core_0_app_init();

  // Launch second core to do USB task handling
  multicore_launch_core1(core1_main);

  core_0_app_task();

  return EXIT_FAILURE;
}