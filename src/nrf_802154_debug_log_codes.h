/* Copyright (c) 2019, Nordic Semiconductor ASA
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

#ifndef NRF_802154_DEBUG_LOG_CODES_H_
#define NRF_802154_DEBUG_LOG_CODES_H_

// Types of log entries.
// Value 0 is reserved and can't be used (reserved for empty log entry)
// Allowed values are in range 1...15
#define NRF_802154_LOG_TYPE_FUNCTION_ENTER 1U
#define NRF_802154_LOG_TYPE_FUNCTION_EXIT  2U
#define NRF_802154_LOG_TYPE_LOCAL_EVENT    3U
#define NRF_802154_LOG_TYPE_GLOBAL_EVENT   4U

// Debug log module identifiers.
// Note: Certain value e.g. NRF_802154_MODULE_ID_RAAL may be shared
// between alternative implementations of module
// (e.g. raal_single_phy/raal_softdevice/raal_simulator)
// In other cases they must be unique.
#define NRF_802154_MODULE_ID_APPLICATION                           1U
#define NRF_802154_MODULE_ID_CORE                                  2U
#define NRF_802154_MODULE_ID_RSCH                                  3U
#define NRF_802154_MODULE_ID_CRITICAL_SECTION                      4U
#define NRF_802154_MODULE_ID_TIMER_COORD                           5U
#define NRF_802154_MODULE_ID_TRX                                   6U
#define NRF_802154_MODULE_ID_TIMER_SCHED                           7U
#define NRF_802154_MODULE_ID_CSMACA                                8U
#define NRF_802154_MODULE_ID_DELAYED_TRX                           9U
#define NRF_802154_MODULE_ID_ACK_TIMEOUT                           10U
#define NRF_802154_MODULE_ID_RAAL                                  11U
#define NRF_802154_MODULE_ID_ANT_DIVERSITY                         12U

// Local events generated by module CORE
#define NRF_802154_LOG_LOCAL_EVENT_ID_CORE_SET_STATE               1U

// Local events generated by module RSCH
#define NRF_802154_LOG_LOCAL_EVENT_ID_RSCH_PRIORITY_SET            1U

// Local events generated by module RAAL
#define NRF_802154_LOG_LOCAL_EVENT_ID_RAAL_TIMESLOT_REQUEST        1U
#define NRF_802154_LOG_LOCAL_EVENT_ID_RAAL_TIMESLOT_REQUEST_RESULT 2U

// Global events
#define NRF_802154_LOG_GLOBAL_EVENT_ID_RADIO_RESET                 1U

#endif /* NRF_802154_DEBUG_LOG_CODES_H_ */
