/**
 * CANopen Indicator specification (CiA 303-3 v1.4.0)
 *
 * @file        CO_LEDs.h
 * @ingroup     CO_LEDs
 * @author      Janez Paternoster
 * @copyright   2020 Janez Paternoster
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#ifndef CO_LEDS_H
#define CO_LEDS_H

#include "301/CO_driver.h"
#include "301/CO_NMT_Heartbeat.h"

/* default configuration, see CO_config.h */
#ifndef CO_CONFIG_LEDS
#define CO_CONFIG_LEDS (CO_CONFIG_LEDS_ENABLE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT|CO_CONFIG_FLAG_TIMERNEXT)
#endif

#if (((CO_CONFIG_LEDS)&CO_CONFIG_LEDS_ENABLE) != 0) || defined CO_DOXYGEN

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_LEDs LED indicators
 * Specified in standard CiA 303-3.
 *
 * @ingroup CO_CANopen_303
 * @{
 * CIA 303-3 standard specifies indicator LED diodes, which reflects state of the CANopen device. Green and red leds or
 * bi-color led can be used.
 *
 * CANopen green led - run led:
 * - flickering: LSS configuration state is active
 * - blinking: device is in NMT pre-operational state
 * - single flash: device is in NMT stopped state
 * - triple flash: a software download is running in the device
 * - on: device is in NMT operational state
 *
 * CANopen red led - error led:
 * - off: no error
 * - flickering:      LSS node id is not configured, CANopen is not initialized
 * - blinking:        invalid configuration, general error
 * - single flash:    CAN warning limit reached
 * - double flash:    heartbeat consumer - error in remote monitored node
 * - triple flash:    sync message reception timeout
 * - quadruple flash: PDO has not been received before the event timer elapsed
 * - on: CAN bus off
 *
 * To apply on/off state to the led diode, use #CO_LED_RED or #CO_LED_GREEN macros with one of the @ref CO_LED_bitmasks.
 * For CANopen leds use #CO_LED_CANopen bitmask.
 */

/**
 * @defgroup CO_LED_bitmasks CO_LED bitmasks
 * @{
 * Bitmasks for the LED indicators
 */
#define CO_LED_flicker               0x01U /**< LED flickering 10Hz */
#define CO_LED_blink                 0x02U /**< LED blinking 2,5Hz */
#define CO_LED_flash_1               0x04U /**< LED single flash */
#define CO_LED_flash_2               0x08U /**< LED double flash */
#define CO_LED_flash_3               0x10U /**< LED triple flash */
#define CO_LED_flash_4               0x20U /**< LED quadruple flash */
#define CO_LED_CANopen               0x80U /**< LED CANopen according to CiA 303-3 */
/** @} */

/** Get on/off state for red led for one of the @ref CO_LED_bitmasks */
#define CO_LED_RED(LEDs, BITMASK)   ((((LEDs)->LEDred & BITMASK) != 0U) ? 1U : 0U)
/** Get on/off state for green led for one of the @ref CO_LED_bitmasks */
#define CO_LED_GREEN(LEDs, BITMASK) ((((LEDs)->LEDgreen & BITMASK) != 0U) ? 1U : 0U)

/**
 * LEDs object, initialized by CO_LEDs_init()
 */
typedef struct {
    uint32_t LEDtmr50ms;   /**< 50ms led timer */
    uint8_t LEDtmr200ms;   /**< 200ms led timer */
    uint8_t LEDtmrflash_1; /**< single flash led timer */
    uint8_t LEDtmrflash_2; /**< double flash led timer */
    uint8_t LEDtmrflash_3; /**< triple flash led timer */
    uint8_t LEDtmrflash_4; /**< quadruple flash led timer */
    uint8_t LEDred;        /**< red led bitfield, to be combined with @ref CO_LED_bitmasks */
    uint8_t LEDgreen;      /**< green led bitfield, to be combined with @ref CO_LED_bitmasks */
} CO_LEDs_t;

/**
 * Initialize LEDs object.
 *
 * Function must be called in the communication reset section.
 *
 * @param LEDs This object will be initialized.
 *
 * @return #CO_ReturnError_t CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_LEDs_init(CO_LEDs_t* LEDs);

/**
 * Process indicator states
 *
 * Function must be called cyclically.
 *
 * @param LEDs This object.
 * @param timeDifference_us Time difference from previous function call in [microseconds].
 * @param NMTstate NMT operating state.
 * @param LSSconfig Node is in LSS configuration state indication.
 * @param ErrCANbusOff CAN bus off indication (highest priority).
 * @param ErrCANbusWarn CAN error warning limit reached indication.
 * @param ErrRpdo RPDO event timer timeout indication.
 * @param ErrSync Sync receive timeout indication.
 * @param ErrHbCons Heartbeat consumer error (remote node) indication.
 * @param ErrOther Other error indication (lowest priority).
 * @param firmwareDownload Firmware download is in progress indication.
 * @param [out] timerNext_us info to OS - see CO_process().
 */
void CO_LEDs_process(
					CO_LEDs_t* LEDs,
					uint32_t timeDifference_us,
					CO_NMT_internalState_t NMTstate,
					bool_t LSSconfig,
					bool_t ErrCANbusOff,
					bool_t ErrCANbusWarn,
					bool_t ErrRpdo,
					bool_t ErrSync,
					bool_t ErrHbCons,
					bool_t ErrOther,
					bool_t firmwareDownload,
					uint32_t* timerNext_us
					);

/** @} */ /* CO_LEDs */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* (CO_CONFIG_LEDS) & CO_CONFIG_LEDS_ENABLE */

#endif /* CO_LEDS_H */
