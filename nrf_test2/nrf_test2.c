// LED blink and serial interface on nRF52832 breakout board
// See iosoft.blog for details

#include <stdarg.h>
#include "nrf_gpio.h"
#include "nrf_drv_systick.h"
#include "nrfx_uart.h"
#include "nrf_fprintf.h"

// LED definitions
#define LED_PIN         7
#define LED_BIT         (1 << LED_PIN)

// UART definitions
#define UART_PIN_DISCONNECTED   0xFFFFFFFF
#define UART_TX_PIN             15
#define UART_RX_PIN             14
#define UART_RTS_PIN            UART_PIN_DISCONNECTED
#define UART_CTS_PIN            UART_PIN_DISCONNECTED
#define UART_BAUD               NRF_UART_BAUDRATE_115200
#define UART_BUFF_SIZE          1024
#define PRINTF_BUFF_SIZE        1024
#define SER_TX_BUFFLEN          1500

uint32_t msticks, systicks;
char printf_buff[PRINTF_BUFF_SIZE];

// Circular buffer for serial Tx
int ser_txin=0, ser_txout=0;
uint8_t ser_txbuff[SER_TX_BUFFLEN];

void init_gpio(void);
int mstimeout(uint32_t *tickp, uint32_t msec);
void init_serial(void);
void poll_serial(void);
int putch(int c);
void uart_event_handler(nrfx_uart_event_t const * evtp, void *ctxp);
void uart_tx(void const *ctxt, char const *buff, size_t len);

// Context for printf function
nrf_fprintf_ctx_t fprintf_ctx = 
{
    .p_io_buffer=printf_buff, .io_buffer_size=PRINTF_BUFF_SIZE,
    .fwrite = uart_tx
};

// Context for UART 0
nrfx_uart_t nrfx_uart = 
{
    NRF_UART0, 0
};

int main(void)
{
    uint32_t tix;

    mstimeout(&tix, 0);
    init_gpio();
    init_serial();
    printf("\nNRF52 test\n");
    while (1)
    {
        if (mstimeout(&tix, 500))
        {
            NRF_GPIO->OUT ^= LED_BIT;
            putch('.');
        }
        poll_serial();
    }
}

// Initialise I/O port, and system tick counter
void init_gpio(void)
{
    nrf_gpio_cfg_output(LED_PIN);
    nrf_drv_systick_init();
}

// Check for timeout in milliseconds
int mstimeout(uint32_t *tickp, uint32_t msec)
{
    uint32_t t, diff;

    t = nrf_systick_val_get();
    diff = NRF_SYSTICK_VAL_MASK & (systicks - t);

    if (msec == 0)
    {
        systicks = t;
        *tickp = msticks;
    }
    else if ((diff /= SystemCoreClock/1000) > 0)
    {
        systicks = t;
        msticks += diff;
        diff = msticks - *tickp;
        if (diff >= msec)
        {
            *tickp = msticks;
            return(1);
        }
    }
    return(0);
}

// Formatted print to serial port
int printf(const char *fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    nrf_fprintf(&fprintf_ctx, fmt, &args);
    n = fprintf_ctx.io_buffer_cnt;
    nrf_fprintf_buffer_flush(&fprintf_ctx);
    va_end(args);
    return(n);
}

// Initialise serial I/O
void init_serial(void)
{
    const nrfx_uart_config_t config = {
        UART_TX_PIN, UART_RX_PIN,
        UART_CTS_PIN, UART_RTS_PIN, 0, 
        NRF_UART_HWFC_DISABLED, 0, UART_BAUD, 
        APP_IRQ_PRIORITY_LOWEST};
    nrfx_uart_init(&nrfx_uart, &config, 0);
}

// Poll serial interface, send next Tx char
void poll_serial(void)
{
    uint8_t b;

    if (ser_txin!=ser_txout && !nrfx_uart_tx_in_progress(&nrfx_uart))
    {
        b = ser_txbuff[ser_txout++];
        if (ser_txout >= SER_TX_BUFFLEN)
            ser_txout = 0;
        nrfx_uart_tx(&nrfx_uart, &b, 1);
    }
}

// Add character to serial Tx circular buffer
// Discard character if buffer is full
int putch(int c)
{
    int in=ser_txin+1;

    if (in >= SER_TX_BUFFLEN)
        in = 0;
    if (in != ser_txout)
    {
        ser_txbuff[ser_txin] = (uint8_t)c;
        ser_txin = in;
    }
    return(c);
}

// Send data block to UART (via circular buffer)
void uart_tx(void const *ctxp, char const *buff, size_t len)
{
    while (len--)
        putch(*buff++);
}

// Dummy UART event handler
void uart_event_handler(nrfx_uart_event_t const * evtp, void *ctxp)
{
}

// EOF
