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

#pragma once

#include "definitions.h"
#include "lz4_bitmaps.h"

DEFINE_LZ4_BITMAP(LBM_POINT);

// calibration bitmaps
extern BitmapBuffer * calibStick;
extern BitmapBuffer * calibStickBackground;

// Channels monitor bitmaps
extern BitmapBuffer * chanMonLockedBitmap;
extern BitmapBuffer * chanMonInvertedBitmap;

void loadBuiltinBitmaps();
const uint8_t* getBuiltinIcon(MenuIcons id);

PACK(struct _bitmap_mask {
  uint16_t w;
  uint16_t h;
  uint8_t mask[0];
});

#define MASK_WIDTH(m) (((_bitmap_mask*)m)->w)
#define MASK_HEIGHT(m) (((_bitmap_mask*)m)->h)
#define MASK_DATA(m) (((_bitmap_mask*)m)->mask)
