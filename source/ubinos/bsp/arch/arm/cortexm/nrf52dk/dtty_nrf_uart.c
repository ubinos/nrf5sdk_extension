/*
 * Copyright (c) 2020 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#if (INCLUDE__UBINOS__BSP == 1)

#if (UBINOS__BSP__USE_DTTY == 1)

#if (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL)

#if (NRF5SDK__DTTY_NRF_UART_ENABLE == 1)

#if (INCLUDE__UBINOS__UBIK != 1)
    #error "ubik is necessary"
#endif

#include <ubinos/bsp.h>
#include <ubinos/bsp/arch.h>
#include <ubinos/bsp_ubik.h>

#include <assert.h>

#include "bsp.h"
#include "nrf_drv_uart.h"

#define SLEEP_TIMEMS	1

extern int _g_bsp_dtty_init;
extern int _g_bsp_dtty_in_init;
extern int _g_bsp_dtty_echo;
extern int _g_bsp_dtty_autocr;

static nrf_drv_uart_t _g_dtty_nrf_uart = NRF_DRV_UART_INSTANCE(0);

cbuf_def_init(_g_dtty_nrf_isr_wbuf, NRF5SDK__DTTY_NRF_ISR_WRITE_BUFFER_SIZE);

cbuf_def_init(_g_dtty_nrf_uart_rbuf, NRF5SDK__DTTY_NRF_UART_READ_BUFFER_SIZE);
cbuf_def_init(_g_dtty_nrf_uart_wbuf, NRF5SDK__DTTY_NRF_UART_WRITE_BUFFER_SIZE);

uint32_t _g_dtty_nrf_uart_rx_overflow_count = 0;
uint8_t _g_dtty_nrf_uart_tx_busy = 0;

sem_pt _g_dtty_nrf_uart_rsem = NULL;

mutex_pt _g_dtty_nrf_uart_putlock = NULL;
mutex_pt _g_dtty_nrf_uart_getlock = NULL;

static int _dtty_getc_advan(char *ch_p, int blocked);

static void dtty_nrf_uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    int need_signal = 0;
    uint8_t * buf;
    uint32_t len;
    nrf_drv_uart_t * uart = &_g_dtty_nrf_uart;
    cbuf_pt rbuf = _g_dtty_nrf_uart_rbuf;
    cbuf_pt wbuf = _g_dtty_nrf_uart_wbuf;
    sem_pt rsem = _g_dtty_nrf_uart_rsem;

    switch (p_event->type)
    {
    case NRF_DRV_UART_EVT_ERROR:
        break;

    case NRF_DRV_UART_EVT_RX_DONE:
        if (p_event->data.rxtx.bytes > 0)
        {
            if (cbuf_is_full(rbuf))
            {
                _g_dtty_nrf_uart_rx_overflow_count++;
                break;
            }

            if (cbuf_get_len(rbuf) == 0)
            {
                need_signal = 1;
            }

            len = 1;
            cbuf_write(rbuf, NULL, len, NULL);

            if (need_signal && _bsp_kernel_active)
            {
                sem_give(rsem);
            }
        }

        buf = cbuf_get_tail_addr(rbuf);
        len = 1;
        nrf_drv_uart_rx(uart, buf, len);
        break;

    case NRF_DRV_UART_EVT_TX_DONE:
        if (p_event->data.rxtx.bytes > 0)
        {
            cbuf_read(wbuf, NULL, p_event->data.rxtx.bytes, NULL);
        }

        if (cbuf_get_len(wbuf) > 0)
        {
            buf = cbuf_get_head_addr(wbuf);
            len = 1;
            nrf_drv_uart_tx(uart, buf, len);
        }
        else
        {
            _g_dtty_nrf_uart_tx_busy = 0;
        }
        break;

    default:
        break;
    }
}

int dtty_init(void)
{
    int r;
    ret_code_t nrf_err;
    uint8_t * buf;
    uint32_t len;
    nrf_drv_uart_config_t config;
    (void) r;
    (void) nrf_err;

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_bsp_kernel_active)
        {
            break;
        }

        if (_g_bsp_dtty_init || _g_bsp_dtty_in_init)
        {
            break;
        }

        _g_bsp_dtty_in_init = 1;

        r = semb_create(&_g_dtty_nrf_uart_rsem);
        assert(r == 0);
        r = mutex_create(&_g_dtty_nrf_uart_putlock);
        assert(r == 0);
        r = mutex_create(&_g_dtty_nrf_uart_getlock);
        assert(r == 0);

        _g_bsp_dtty_echo = 1;
        _g_bsp_dtty_autocr = 1;

        config.pseltxd = TX_PIN_NUMBER;
        config.pselrxd = RX_PIN_NUMBER;
        config.pselcts = CTS_PIN_NUMBER;
        config.pselrts = RTS_PIN_NUMBER;
        config.p_context = NULL;
        config.hwfc = NRF_UART_HWFC_DISABLED;
        config.parity = NRF_UART_PARITY_EXCLUDED;
        config.baudrate = NRF_UART_BAUDRATE_115200;
        config.interrupt_priority = NVIC_PRIO_LOWEST;
#if defined(NRF_DRV_UART_WITH_UARTE) && defined(NRF_DRV_UART_WITH_UART)
        config.use_easy_dma = true;
#endif
        nrf_err = nrf_drv_uart_init(&_g_dtty_nrf_uart, &config, dtty_nrf_uart_event_handler);
        assert(nrf_err == NRF_SUCCESS);

        _g_bsp_dtty_init = 1;

        cbuf_clear(_g_dtty_nrf_uart_rbuf);

        buf = cbuf_get_tail_addr(_g_dtty_nrf_uart_rbuf);
        len = 1;
        nrf_drv_uart_rx(&_g_dtty_nrf_uart, buf, len);

        _g_bsp_dtty_in_init = 0;

        break;
    } while (1);

    return 0;
}

int dtty_enable(void)
{
    return 0;
}

int dtty_disable(void)
{
    return 0;
}

int dtty_geterror(void)
{
    return 0;
}

static int _dtty_getc_advan(char *ch_p, int blocked)
{
    int r;
    ubi_err_t ubi_err;

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        if (!blocked)
        {
            r = mutex_lock_timed(_g_dtty_nrf_uart_getlock, 0);
        }
        else
        {
            r = mutex_lock(_g_dtty_nrf_uart_getlock);
        }
        if (r != 0)
        {
            break;
        }

        for (;;)
        {
            ubi_err = cbuf_read(_g_dtty_nrf_uart_rbuf, (uint8_t*) ch_p, 1, NULL);
            if (ubi_err == UBI_ERR_OK)
            {
                r = 0;
                break;
            }
            else
            {
                if (!blocked)
                {
                    break;
                }
                else
                {
                    sem_take(_g_dtty_nrf_uart_rsem);
                }
            }
        }

        if (0 == r && 0 != _g_bsp_dtty_echo)
        {
            dtty_putc(*ch_p);
        }

        mutex_unlock(_g_dtty_nrf_uart_getlock);

        break;
    } while (1);

    return r;
}

int dtty_getc(char *ch_p)
{
    return _dtty_getc_advan(ch_p, 1);
}

int dtty_getc_unblocked(char *ch_p)
{
    return _dtty_getc_advan(ch_p, 0);
}

int dtty_putc(int ch)
{
    int r;
    uint8_t * buf;
    size_t len;
    uint32_t written;
    uint8_t data[2];

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            data[0] = (uint8_t) ch;
            len = 1;
            cbuf_write(_g_dtty_nrf_isr_wbuf, data, len, &written);

            r = 0;
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        mutex_lock(_g_dtty_nrf_uart_putlock);

        do
        {
            if (0 != _g_bsp_dtty_autocr && '\n' == ch)
            {
                data[0] = '\r';
                data[1] = '\n';
                len = 2;
            }
            else
            {
                data[0] = (uint8_t) ch;
                len = 1;
            }

            cbuf_write(_g_dtty_nrf_uart_wbuf, data, len, &written);
            if (written == 0)
            {
                break;
            }

            if (!_g_dtty_nrf_uart_tx_busy)
            {
                buf = cbuf_get_head_addr(_g_dtty_nrf_uart_wbuf);
                len = 1;
                _g_dtty_nrf_uart_tx_busy = 1;
                for (uint32_t i = 0;; i++)
                {
                    if (!nrf_drv_uart_tx_in_progress(&_g_dtty_nrf_uart))
                    {
                        nrf_drv_uart_tx(&_g_dtty_nrf_uart, buf, len);
                        break;
                    }
                    if (i >= 99)
                    {
                        break;
                    }
                }
            }

            r = 0;
            break;
        } while (1);

        mutex_unlock(_g_dtty_nrf_uart_putlock);

        break;
    } while (1);

    return r;
}

int dtty_flush(void)
{
    return 0;
}

int dtty_putn(const char *str, int len)
{
    int r;
    uint32_t written;

    r = -1;
    do
    {
        if (NULL == str)
        {
            r = -2;
            break;
        }

        if (0 > len)
        {
            r = -3;
            break;
        }

        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            cbuf_write(_g_dtty_nrf_isr_wbuf, (uint8_t *) str, len, &written);
            r = written;
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        for (r = 0; r < len; r++)
        {
            dtty_putc(*str);
            str++;
        }

        break;
    } while (1);

    return r;
}

int dtty_kbhit(void)
{
    int r;

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        if (cbuf_get_len(_g_dtty_nrf_uart_rbuf) != 0)
        {
            r = 1;
        }
        else
        {
            r = 0;
        }

        break;
    } while (1);

    return r;
}

void dtty_isr_write_process(void *arg)
{
    uint8_t * buf;
    uint32_t len;
    cbuf_pt wbuf = _g_dtty_nrf_isr_wbuf;

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        while (cbuf_get_len(wbuf) > 0)
        {
            buf = cbuf_get_head_addr(wbuf);
            len = cbuf_get_contig_len(wbuf);
            dtty_putn((const char *) buf, len);
            cbuf_read(wbuf, NULL, len, NULL);
        }

        break;
    } while (1);
}

#endif /* (NRF5SDK__DTTY_NRF_UART_ENABLE == 1) */

#endif /* (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL) */

#endif /* (UBINOS__BSP__USE_DTTY == 1) */

#endif /* (INCLUDE__UBINOS__BSP == 1) */

