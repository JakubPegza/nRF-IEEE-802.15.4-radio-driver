/* Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *   This file implements the 802.15.4 antenna diversity module.
 *
 */
#include <assert.h>

#include "nrf_802154_ant_div.h"
#include "nrf_error.h"
#include "nrf_gpio.h"

#ifdef ENABLE_ANT_DIV

static nrf_802154_ant_div_config_t m_ant_div_config =      /**< Antenna Diversity configuration */
{
    .ant_sel_pin = NRF_ANT_DIV_ANT_SEL_DEFAULT_PIN
};

int32_t nrf_802154_ant_div_init()
{
    nrf_gpio_cfg_output(m_ant_div_config.ant_sel_pin);
    return NRF_SUCCESS;
}

int32_t nrf_802154_ant_div_set_config(nrf_802154_ant_div_config_t *p_ant_div_config)
{
    assert(p_ant_div_config != NULL);
    m_ant_div_config = *p_ant_div_config;
    return NRF_SUCCESS;
}

int32_t nrf_802154_ant_div_get_config(nrf_802154_ant_div_config_t *p_ant_div_config)
{
    assert(p_ant_div_config != NULL);
    *p_ant_div_config = m_ant_div_config;
    return NRF_SUCCESS;
}

int32_t nrf_802154_ant_div_set_antenna(nrf_802154_ant_div_antenna_t antenna)
{
    nrf_gpio_pin_write(m_ant_div_config.ant_sel_pin, antenna);
    return NRF_SUCCESS;
}

int32_t nrf_802154_ant_div_get_antenna(nrf_802154_ant_div_antenna_t *antenna)
{
    *antenna = (NRF_GPIO->OUT >> m_ant_div_config.ant_sel_pin) & 0x01;
    return NRF_SUCCESS;
}
#else //ENABLE_ANT_DIV

int32_t nrf_802154_ant_div_init()
{
    return NRF_ERROR_NOT_SUPPORTED;
}

int32_t nrf_802154_ant_div_set_config(nrf_802154_ant_div_config_t *p_ant_div_config)
{
    (void)p_ant_div_config;
    return NRF_ERROR_NOT_SUPPORTED;
}

int32_t nrf_802154_ant_div_get_config(nrf_802154_ant_div_config_t *p_ant_div_config)
{
    (void)p_ant_div_config;
    return NRF_ERROR_NOT_SUPPORTED;
}

int32_t nrf_802154_ant_div_set_antenna(nrf_802154_ant_div_antenna_t antenna)
{
    (void)antenna;
    return NRF_ERROR_NOT_SUPPORTED;
}

int32_t nrf_802154_ant_div_get_antenna(nrf_802154_ant_div_antenna_t *antenna)
{
    (void)antenna;
    return NRF_ERROR_NOT_SUPPORTED;
}

#endif //ENABLE_ANT_DIV