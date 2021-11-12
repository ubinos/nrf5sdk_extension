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
#include <ubinos/bsp_ubik.h>

#include <assert.h>

#include <boards.h>

#include "nrf_libuarte_async.h"

#define SLEEP_TIMEMS	1

extern int _g_bsp_dtty_init;
extern int _g_bsp_dtty_in_init;
extern int _g_bsp_dtty_echo;
extern int _g_bsp_dtty_autocr;

#define DTTY_NRF_LIBUARTE_READ_BUFFER_SIZE (512)
#define DTTY_NRF_LIBUARTE_WRITE_BUFFER_SIZE (1024 * 10)

NRF_LIBUARTE_ASYNC_DEFINE(_g_dtty_nrf_libuarte, 0, 0, 0, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 3);

cbuf_def_init(_g_dtty_nrf_libuarte_rbuf, DTTY_NRF_LIBUARTE_READ_BUFFER_SIZE);
cbuf_def_init(_g_dtty_nrf_libuarte_wbuf, DTTY_NRF_LIBUARTE_WRITE_BUFFER_SIZE);

uint8_t _g_dtty_nrf_libuarte_overflow = 0;
uint8_t _g_dtty_nrf_libuarte_is_tx_busy = 0;

sem_pt _g_dtty_nrf_libuarte_read_sem = NULL;

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
            _g_dtty_nrf_libuarte_overflow = 1;
            break;
        }

        if (cbuf_get_len(_g_dtty_nrf_libuarte_rbuf) == 0)
        {
            need_signal = 1;
        }

        cbuf_write(_g_dtty_nrf_libuarte_rbuf, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length, NULL);

        if (need_signal && _bsp_kernel_active)
        {
            sem_give(_g_dtty_nrf_libuarte_read_sem);
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
            _g_dtty_nrf_libuarte_is_tx_busy = 0;
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

    if (!_g_bsp_dtty_init && !_g_bsp_dtty_in_init && !bsp_isintr() && _bsp_kernel_active)
    {
        _g_bsp_dtty_in_init = 1;

        r = semb_create(&_g_dtty_nrf_libuarte_read_sem);
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

        _g_bsp_dtty_in_init = 0;
    }

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

int dtty_getc(char *ch_p)
{
    ubi_err_t ubi_err;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    if (_g_bsp_dtty_init && !bsp_isintr())
    {
        while (1)
        {
            if (cbuf_get_len(_g_dtty_nrf_libuarte_rbuf) == 0)
            {
                sem_take(_g_dtty_nrf_libuarte_read_sem);
            }

            ubi_err = cbuf_read(_g_dtty_nrf_libuarte_rbuf, (uint8_t*) ch_p, 1, NULL);
            if (ubi_err == UBI_ERR_OK)
            {
                break;
            }
        }
    }

    if (0 != _g_bsp_dtty_echo)
    {
        dtty_putc(*ch_p);
    }

    return 0;
}

int dtty_putc(int ch)
{
    uint32_t written;
    uint8_t * buf;
    size_t len;
    uint8_t data[2];
    int ret;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    ret = 0;
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
        ret = 1;

        if (_g_bsp_dtty_init && !bsp_isintr())
        {
            if (!_g_dtty_nrf_libuarte_is_tx_busy) {
                buf = cbuf_get_head_addr(_g_dtty_nrf_libuarte_wbuf);
                len = cbuf_get_contig_len(_g_dtty_nrf_libuarte_wbuf);
                _g_dtty_nrf_libuarte_is_tx_busy = 1;
                while (1)
                {
                    ret = nrf_libuarte_async_tx(&_g_dtty_nrf_libuarte, buf, len);
                    if (ret == NRF_ERROR_BUSY)
                    {
                        continue;
                    }
                    APP_ERROR_CHECK(ret);
                    break;
                }
            }
        }
    } while (0);

    return ret;



    if (_g_bsp_dtty_init && !bsp_isintr())
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

        while (1)
        {
            ret = nrf_libuarte_async_tx(&_g_dtty_nrf_libuarte, data, len);
            if (ret == NRF_ERROR_BUSY)
            {
                continue;
            }
            APP_ERROR_CHECK(ret);

            break;
        }
    }

    return ret;
}

int dtty_flush(void)
{
    return 0;
}

int dtty_putn(const char *str, int len)
{
    int i = 0;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    if (_g_bsp_dtty_init && !bsp_isintr())
    {
        if (NULL == str)
        {
            return -2;
        }

        if (0 > len)
        {
            return -3;
        }

        for (i = 0; i < len; i++)
        {
            dtty_putc(*str);
            str++;
        }
    }
    return i;
}

int dtty_kbhit(void)
{
    int r = 0;

    if (!_g_bsp_dtty_init)
    {
        dtty_init();
    }

    if (_g_bsp_dtty_init && !bsp_isintr())
    {
        if (cbuf_get_len(_g_dtty_nrf_libuarte_rbuf) != 0)
        {
            r = 1;
        }
        else
        {
            r = 0;
        }
    }

    return r;
}

#endif /* (NRF5SDK__DTTY_NRF_LIBUARTE_ENABLE == 1) */

#endif /* (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL) */

#endif /* (UBINOS__BSP__USE_DTTY == 1) */

#endif /* (INCLUDE__UBINOS__BSP == 1) */

