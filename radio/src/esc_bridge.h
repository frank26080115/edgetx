/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ESC_BRIDGE_H_
#define _ESC_BRIDGE_H_

#include <stdint.h>
#include <stdbool.h>

//#include "opentx.h"

#include "serial.h"

// return values for initialization functions
enum {
    ESCBRIDGE_INIT_SUCCESS     =  0,
    ESCBRIDGE_INIT_ERR_NO_DRV  = -1,
    ESCBRIDGE_INIT_ERR_NO_CTX  = -2,
    ESCBRIDGE_INIT_ERR_NO_MOD  = -3,
    ESCBRIDGE_INIT_ERR_NO_PORT = -4,
    ESCBRIDGE_INIT_ERR_NO_AUX  = -5,
    ESCBRIDGE_INIT_ERR_NO_HW   = -6,
    ESCBRIDGE_INIT_ERR_UNABLE  = -7,
    ESCBRIDGE_INIT_ERR_SECOND  = -20,
};

void escBridgeTx(uint8_t* buf, uint32_t len); // replacement for the CLI data received callback
int16_t escBridgeReadByte();  // returns one byte read from UART, or -1 for empty buffer

// various port initializers, returns ESCBRIDGE_INIT_x, 0 on success, negative number for error code
int8_t escBridgeTrainerInit(const etx_serial_init* params);
int8_t escBridgeExtModInit(const etx_serial_init* params);
int8_t escBridgeAuxInit(uint8_t port_n, const etx_serial_init* params);
int8_t escBridgeSportInit(etx_serial_init* params);
int8_t escBridgeJrBayInit(etx_serial_init* params);

// show text on screen
void escBridgeDisplayLcd();

#endif // _ESC_BRIDGE_H_
