/*
 * Copyright (c) 2020 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#if (INCLUDE__UBINOS__BSP == 1)

#if (UBINOS__BSP__USE_DTTY == 1)

#if (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL)

#if (NRF5SDK__DTTY_NRF_LIBUARTE_ENABLE == 1)

#if (INCLUDE__UBINOS__UBIK != 1)
    #error "ubik is necessary"
#endif

#include <ubinos/bsp.h>
#include <ubinos/bsp/arch.h>
#include <ubinos/bsp_ubik.h>

#include <assert.h>

#include "bsp.h"
#include "nrf_libuarte_async.h"

#define SLEEP_TIMEMS	1

extern int _g_bsp_dtty_init;
extern int _g_bsp_dtty_in_init;
extern int _g_bsp_dtty_echo;
extern int _g_bsp_dtty_autocr;

NRF_LIBUARTE_ASYNC_DEFINE(_g_dtty_nrf_libuarte, 0, 0, 0, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 3);

cbuf_def_init(_g_dtty_nrf_libuarte_rbuf, NRF5SDK__DTTY_NRF_LIBUARTE_READ_BUFFER_SIZE);
cbuf_def_init(_g_dtty_nrf_libuarte_wbuf, NRF5SDK__DTTY_NRF_LIBUARTE_WRITE_BUFFER_SIZE);

uint32_t _g_dtty_nrf_libuarte_rx_overflow_count = 0;
uint8_t _g_dtty_nrf_libuarte_tx_busy = 0;

sem_pt _g_dtty_nrf_libuarte_rsem = NULL;

mutex_pt _g_dtty_nrf_libuarte_putlock = NULL;
mutex_pt _g_dtty_nrf_libuarte_getlock = NULL;

static int _dtty_getc_advan(char *ch_p, int blocked);

static void dtty_nrf_libuarte_event_handler(void *context, nrf_libuarte_async_evt_t *p_evt)
{
    int need_signal = 0;
    uint8_t * buf;
    uint32_t len;

    switch (p_evt->type)
    {
    case NRF_LIBUARTE_ASYNC_EVT_ERROR:
        break;

    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
        if (p_evt->data.rxtx.length == 0)
        {
            break;
        }

        if (cbuf_is_full(_g_dtty_nrf_libuarte_rbuf))
        {
            _g_dtty_nrf_libuarte_rx_overflow_count = 1;
            break;
        }

        if (cbuf_get_len(_g_dtty_nrf_libuarte_rbuf) == 0)
        {
            need_signal = 1;
        }

        cbuf_write(_g_dtty_nrf_libuarte_rbuf, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length, NULL);

        if (need_signal && _bsp_kernel_active)
        {
            sem_give(_g_dtty_nrf_libuarte_rsem);
        }
        break;

    case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
        if (p_evt->data.rxtx.length > 0)
        {
            cbuf_read(_g_dtty_nrf_libuarte_wbuf, NULL, p_evt->data.rxtx.length, NULL);
        }

        if (cbuf_get_len(_g_dtty_nrf_libuarte_wbuf) > 0)
        {
            buf = cbuf_get_head_addr(_g_dtty_nrf_libuarte_wbuf);
            len = cbuf_get_contig_len(_g_dtty_nrf_libuarte_wbuf);
            nrf_libuarte_async_tx(&_g_dtty_nrf_libuarte, buf, len);
        }
        else {
            _g_dtty_nrf_libuarte_tx_busy = 0;
        }
        break;

    default:
        break;
    }
}

int dtty_init(void)
{
    int r;
    ret_code_t err_code;

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

        r = semb_create(&_g_dtty_nrf_libuarte_rsem);
        assert(r == 0);
        r = mutex_create(&_g_dtty_nrf_libuarte_putlock);
        assert(r == 0);
        r = mutex_create(&_g_dtty_nrf_libuarte_getlock);
        assert(r == 0);

        _g_bsp_dtty_echo = 1;
        _g_bsp_dtty_autocr = 1;

        nrf_libuarte_async_config_t nrf_libuarte_async_config =
        {
                .tx_pin = TX_PIN_NUMBER,
                .rx_pin = RX_PIN_NUMBER,
                .baudrate = NRF_UARTE_BAUDRATE_115200,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .timeout_us = 100,
                .int_prio = APP_IRQ_PRIORITY_LOW };

        err_code = nrf_libuarte_async_init(&_g_dtty_nrf_libuarte, &nrf_libuarte_async_config, dtty_nrf_libuarte_event_handler, (void*) &_g_dtty_nrf_libuarte);
        APP_ERROR_CHECK(err_code);

        nrf_libuarte_async_enable(&_g_dtty_nrf_libuarte);

        _g_bsp_dtty_init = 1;

        cbuf_clear(_g_dtty_nrf_libuarte_rbuf);

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
            r = mutex_lock_timed(_g_dtty_nrf_libuarte_getlock, 0);
        }
        else
        {
            r = mutex_lock(_g_dtty_nrf_libuarte_getlock);
        }
        if (r != 0)
        {
            break;
        }

        for (;;)
        {
            ubi_err = cbuf_read(_g_dtty_nrf_libuarte_rbuf, (uint8_t*) ch_p, 1, NULL);
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
                    sem_take(_g_dtty_nrf_libuarte_rsem);
                }
            }
        }

        if (0 == r && 0 != _g_bsp_dtty_echo)
        {
            dtty_putc(*ch_p);
        }

        mutex_unlock(_g_dtty_nrf_libuarte_getlock);

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
    uint32_t written;
    uint8_t * buf;
    size_t len;
    uint8_t data[2];

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

        mutex_lock(_g_dtty_nrf_libuarte_putlock);

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

            cbuf_write(_g_dtty_nrf_libuarte_wbuf, data, len, &written);
            if (written == 0)
            {
                break;
            }

            if (!_g_dtty_nrf_libuarte_tx_busy) {
                buf = cbuf_get_head_addr(_g_dtty_nrf_libuarte_wbuf);
                len = cbuf_get_contig_len(_g_dtty_nrf_libuarte_wbuf);
                _g_dtty_nrf_libuarte_tx_busy = 1;
                while (1)
                {
                    r = nrf_libuarte_async_tx(&_g_dtty_nrf_libuarte, buf, len);
                    if (r == NRF_ERROR_BUSY)
                    {
                        continue;
                    }
                    APP_ERROR_CHECK(r);
                    break;
                }
            }

            r = 0;
            break;
        } while (1);

        mutex_unlock(_g_dtty_nrf_libuarte_putlock);

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

        if (cbuf_get_len(_g_dtty_nrf_libuarte_rbuf) != 0)
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

#endif /* (NRF5SDK__DTTY_NRF_LIBUARTE_ENABLE == 1) */

#endif /* (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL) */

#endif /* (UBINOS__BSP__USE_DTTY == 1) */

#endif /* (INCLUDE__UBINOS__BSP == 1) */

