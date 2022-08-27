#ifndef AP_UART_H
#define AP_UART_H

#include <hardware/uart.h>

#define LED_PIN_NUMBER 25


#define NUM_HARD_UARTS 2
#define NUM_PIO_UARTS 2

#define NUM_APP_UARTS (NUM_HARD_UARTS + NUM_PIO_UARTS)


// UARTs 0 and 1 have baud rates that can be configured, therefore they don't
// have individual baud rates. PIO configurable baud rate needs testing.
#define UART0_ID_NUM 0
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

#define UART1_ID_NUM 1
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

#define UART2_ID_NUM 2
#define UART2_TX_PIN 2
#define UART2_RX_PIN 3
#define UART2_BAUD_RATE DEFAULT_BAUD_RATE

#define UART3_ID_NUM 3
#define UART3_TX_PIN 6
#define UART3_RX_PIN 7
#define UART3_BAUD_RATE DEFAULT_BAUD_RATE

// 4 state machines per PIO
// In this app, SM is tied to UART. eg PIO0-SM0 and PIO1-SM0 would be PIO_UART0)
#define PIO_SM0_NUM 0
#define PIO_SM1_NUM 1
#define PIO_SM2_NUM 2
#define PIO_SM3_NUM 3


// Must be unique for each UART instance (across hardware and pio).
typedef uint8_t uart_uid;

typedef struct {
    uart_uid uid;
	uart_inst_t *const hard_inst;
	uint8_t hard_tx_pin;
	uint8_t hard_rx_pin;
} hard_uart_id_t;

typedef struct {
    uart_uid uid;
    uint8_t pio_sm_num; // PIO state machine number (both RX and TX)
    uint8_t pio_sm_tx_pin;
    uint8_t pio_sm_rx_pin;
} pio_uart_id_t;

#define BUFFER_SIZE 1024

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	mutex_t usb_mtx;
} uart_data_t;


// USB CDC line coding options
typedef enum {
  USB_CDC_LC_FIVE_DBITS = 5,
  USB_CDC_LC_SIX_DBITS = 6,
  USB_CDC_LC_SEVEN_DBITS = 7,
  USB_CDC_LC_EIGHT_DBITS = 8,
  USB_CDC_LC_SIXTEEN_DBITS = 16,
} usb_cdc_lc_databits_t;

typedef enum {
  USB_CDC_LC_STOP_BIT_1 = 0,
  USB_CDC_LC_STOP_BIT_1_5 = 1,
  USB_CDC_LC_STOP_BIT_2 = 2,
} usb_cdc_lc_stopbits_t;

typedef enum {
  USB_CDC_LC_NO_PARITY = 0,
  USB_CDC_LC_ODD_PARITY = 1,
  USB_CDC_LC_EVEN_PARITY = 2,
  USB_CDC_LC_MARK_PARITY = 3,
  USB_CDC_LC_SPACE_PARITY = 4,
} usb_cdc_lc_parity_t;


// Raspberry Pi Pico UART line coding options
// Note: parity enum already defined in uart.h
typedef enum { 
  UART_FIVE_DBITS = 5,
  UART_SIX_DBITS = 6,
  UART_SEVEN_DBITS = 7,
  UART_EIGHT_DBITS = 8,
} uart_databits_t;

typedef enum {
  UART_STOP_BIT_1 = 1,
  UART_STOP_BIT_2 = 2,
} uart_stopbits_t;


#define DEFAULT_BAUD_RATE 115200

#define DEFAULT_UART_NUM_STOP_BITS UART_STOP_BIT_1
#define DEFAULT_UART_NUM_DATA_BITS UART_EIGHT_DBITS
#define DEFAULT_UART_PARITY_MODE UART_PARITY_NONE

#define DEFAULT_USB_CDC_NUM_STOP_BITS USB_CDC_LC_STOP_BIT_1
#define DEFAULT_USB_CDC_NUM_DATA_BITS USB_CDC_LC_EIGHT_DBITS
#define DEFAULT_USB_CDC_PARITY_MODE USB_CDC_LC_NO_PARITY

#endif