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

#include "opentx.h"

#include "serial.h"

void escBridgeTx(uint8_t* buf, uint32_t len); // replacement for the CLI data received callback
int16_t escBridgeReadByte();  // returns one byte read from UART, or -1 for empty buffer

// various port initializers
void escBridgeTrainerInit(const etx_serial_init* params);
void escBridgeExtModInit(const etx_serial_init* params);
void escBridgeAuxInit(uint8_t port_n, const etx_serial_init* params);
void escBridgeSportInit(const etx_serial_init* params);

#endif // _ESC_BRIDGE_H_
