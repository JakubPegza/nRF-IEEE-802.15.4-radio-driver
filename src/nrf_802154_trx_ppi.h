/*
 * Copyright (c) 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @brief Module that contains PPI management functions for
 *        the nRF IEEE 802.15.4 radio driver.
 */

#ifndef NRF_802154_TRX_PPI_H_
#define NRF_802154_TRX_PPI_H_

#include <stdbool.h>
#include <stdint.h>

#include "hal/nrf_radio.h"

/**
 * @brief Clear all PPIs configured by this module.
 *
 * Cleared PPIs are used:
 *  - to connect RADIO DISABLED event with tasks needed to ramp up
 *  - by FEM
 *  - to enable RADIO SYNC event IRQ
 */
void nrf_802154_trx_ppi_all_clear(void);

/**
 * @brief Set PPIs to connect RADIO DISABLED event with tasks needed to ramp up.
 *
 * @param[in]  ramp_up_task  Task triggered to start ramp up procedure.
 * @param[in]  start_timer   If timer is started on RADIO DISABLED event.
 */
void nrf_802154_trx_ppi_for_ramp_up_set(nrf_radio_task_t ramp_up_task, bool start_timer);

/**
 * @brief Clear PPIs to connect RADIO DISABLED event with tasks needed to ramp up.
 *
 * @param[in]  start_timer   If timer is started on RADIO DISABLED event.
 */
void nrf_802154_trx_ppi_for_ramp_up_clear(bool start_timer);

/**
 * @brief Wait until PPIs configured to ramp up radio are propagated through PPI system.
 */
void nrf_802154_trx_ppi_for_ramp_up_propagation_delay_wait(void);

/**
 * @brief Detect if PPIs configured to start radio operation were triggered.
 *
 * Radio ramp up starts by design from RADIO DISABLED event. This functions verifies this event
 * and PPIs status.
 *
 * @retval  true   PPIs were triggered.
 * @retval  false  PPIs were not triggered. RADIO DISABLE task shall be called to trigger them.
 */
bool nrf_802154_trx_ppi_for_ramp_up_was_triggered(void);

/**
 * @brief Set PPIs to connect TIMER event with tasks needed to ramp up ACK TX.
 */
void nrf_802154_trx_ppi_for_ack_tx_set(void);

/**
 * @brief Clear PPIs to connect TIMER event with tasks needed to ramp up ACK TX.
 */
void nrf_802154_trx_ppi_for_ack_tx_clear(void);

/**
 * @brief Configure PPIs needed for external LNA or PA.
 */
void nrf_802154_trx_ppi_for_fem_set(void);

/**
 * @brief Unconfigure PPIs needed for external LNA or PA.
 */
void nrf_802154_trx_ppi_for_fem_clear(void);

/**
 * @brief Prepare FEM to enter powerdown state.
 *
 * @param[in] p_instance Timer instance that is used to schedule the transition to the Power Down state.
 * @param[in] compare_channel Compare channel to hold a value for the timer.
 *
 * @retval true   FEM powerdown procedure has started.
 * @retval false  FEM powerdown procedure is not needed.
 */
bool nrf_802154_trx_ppi_for_fem_powerdown_set(NRF_TIMER_Type * p_instance,
                                              uint32_t         compare_channel);

/**
 * @brief Unconfigure PPIs needed to enter the powerdown state by FEM.
 */
void nrf_802154_trx_ppi_for_fem_powerdown_clear(void);

/**
 * @brief Get PPI group id that is disalbed when external event aborts radio operation.
 */
uint32_t nrf_802154_trx_ppi_group_for_abort_get(void);

/**
 * @brief Configure PPIs needed to trigger IRQ from RADIO event SYNC.
 */
void nrf_802154_trx_ppi_for_radio_sync_set(uint32_t task);

/**
 * @brief Unconfigure PPIs needed to trigger IRQ from RADIO event SYNC.
 */
void nrf_802154_trx_ppi_for_radio_sync_clear(void);

#endif /* NRF_802154_TRX_PPI_H_ */
