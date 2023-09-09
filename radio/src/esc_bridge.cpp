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

/*
 * This code module implements a half duplex UART bridge
 * that utilizes existing UART pins on the radio, accessible to the user.
 * It is intended to turn the radio into a tool that can update brushless ESCs
 * with firmware such as BLHeli_32 or AM32.
 * 
 * The wiring to the ESC is different depending on the UART being used.
 * As most of the UARTs are only implemented with two pins, the code here
 * will enable and disable the TX pin as needed.
 * 
 * This feature is activated through the CLI of EdgeTx. Once activated,
 * the appropriate PC app can be used
 * 
 * To activate this feature in the firmware build:
 * CLI must be enabled and ENABLE_SERIAL_PASSTHROUGH must be enabled
 * Available ports are auto determined by the target HW definitions
 * 
 * To use this feature:
 * Select CLI as the USB-VCP's mode
 * Plug in the USB cable, select VCP mode
 * Open a terminal app on the host PC, type in the command
 *     serialpassthrough escbridge <port> <baud>
 * such as
 *     serialpassthrough escbridge aux1 19200
 * Then close the terminal app and use the ESC's dedicated PC app
*/

#include "hal.h"
//#include "dataconstants.h" // causes compiler errors
//#include "myeeprom.h" // causes compiler errors
#include "serial.h"
#include "hal/module_port.h"
#include "stm32_usart_driver.h"
#include "stm32_serial_driver.h"
#include "stm32_hal.h"

#include <stdlib.h>

#if defined(TRAINER_MODULE_SBUS_USART)
void* trainerGetContext();
void init_trainer_module_sbus();
#endif

// return values for initialization functions
enum {
    ESCBRIDGE_INIT_SUCCESS     =  0,
    ESCBRIDGE_INIT_ERR_NO_DRV  = -1,
    ESCBRIDGE_INIT_ERR_NO_CTX  = -2,
    ESCBRIDGE_INIT_ERR_NO_MOD  = -3,
    ESCBRIDGE_INIT_ERR_NO_PORT = -4,
    ESCBRIDGE_INIT_ERR_NO_AUX  = -5,
    ESCBRIDGE_INIT_ERR_NO_HW   = -6,
};

etx_serial_port_t* escBridgePort = nullptr;
etx_serial_driver_t* escBridgeDrv = nullptr;
etx_module_state_t* escBridgeModState = nullptr;
void* escBridgeCtx = nullptr;
uint32_t escBridgeTxPin = 0;
GPIO_TypeDef* escBridgeTxGpiox = nullptr;
// TODO: using raw STM32 items here makes this code not portable to other microcontroller families
// suggested action is to add a driver->setTxPinDir(bool isOut) function
// but that change is very invasive, as it would require data structure changes to seperate TX pin and RX pin

bool escBridgeHasDirPin = false;    // indicates that direction change will happen automatically
bool escBridgeRequireEcho = false;  // implementations of half-duplex will disable RX while TX, so a manual echo is needed

// if manual echo is required, then implement a FIFO to store all the bytes to echo out
#define ESCBRIDGE_FIFO_SIZE    512
static uint8_t* fifo = nullptr;
static int16_t fifo_w = 0, fifo_r = 0;
static int16_t fifo_sz = 0;

static void escBridgeInitFifo()
{
  // FIFO buffer dynamically allocated because not everybody is going to use ESC Bridge
  fifo = (uint8_t*)malloc(fifo_sz = ESCBRIDGE_FIFO_SIZE);
  escBridgeRequireEcho = true;
}

static inline void escBridgeTxPinSetDir(bool isOut)
{
  // this assumes the pin has its alternate-function already assigned from the serial port initialization
  if (escBridgeHasDirPin == false) {
    LL_GPIO_SetPinMode(escBridgeTxGpiox, escBridgeTxPin, isOut ? LL_GPIO_MODE_ALTERNATE : LL_GPIO_MODE_INPUT);
  }
  if (isOut == false) {
    escBridgeDrv->enableRx(escBridgeCtx);
  }
}

// replacement for the CLI data received callback
void escBridgeTx(uint8_t* buf, uint32_t len)
{
  if (!escBridgeDrv) { // loopback echo
    if (!fifo) {
      fifo = (uint8_t*)malloc(fifo_sz = ESCBRIDGE_FIFO_SIZE);
    }
    while (len > 0) {
      // push all data from buf into fifo
      fifo[fifo_w] = *(buf++);
      fifo_w = (fifo_w + 1) % fifo_sz;
      len--;
    }
    return;
  }

  escBridgeTxPinSetDir(true); // make the pin take over as transmitter
  while (len > 0) {
    // send all bytes from VCP to UART
    uint8_t x = *(buf++);
    escBridgeDrv->sendByte(escBridgeCtx, x);
    len--;

    if (escBridgeRequireEcho && fifo) {
      // push all data from buf into fifo, for manual echo
      fifo[fifo_w] = x;
      fifo_w = (fifo_w + 1) % fifo_sz;
    }
  }
  escBridgeDrv->waitForTxCompleted(escBridgeCtx); // need to wait until all the bytes are sent physically, then the TX pin must be turned into an input
  escBridgeTxPinSetDir(false); // make the pin not drive the line, so that the ESC can reply
}

// the code that sends data from the USART to VCP is in cli.cpp right inside the CLI loop
// this function is what fetches the byte
int16_t escBridgeReadByte()
{
  uint8_t data;
  if (fifo_r != fifo_w && fifo) { // data available in fifo
    // pop from fifo
    data = fifo[fifo_r];
    fifo_r = (fifo_r + 1) % fifo_sz;
    return (int16_t)data;
  }
  if (!escBridgeDrv) { // no data if driver is missing
    return -1;
  }
  if (escBridgeDrv->getByte(escBridgeCtx, &data) > 0) { // yes data
    return (int16_t)data;
  }
  return -1; // no data
}

int8_t escBridgeTrainerInit(const etx_serial_init* params)
{
  #if defined(TRAINER_MODULE_SBUS_USART)
  escBridgeDrv = (etx_serial_driver_t*)&STM32SerialDriver;
  escBridgeCtx = trainerGetContext();
  if (!escBridgeCtx) {
    // if it was not initialized, then initialize it
    init_trainer_module_sbus();
    escBridgeCtx = trainerGetContext();
  }
  if (!escBridgeCtx) {
    return ESCBRIDGE_INIT_ERR_NO_CTX;
  }
  escBridgeDrv->setBaudrate(escBridgeCtx, params->baudrate);
  escBridgeTxPin = TRAINER_MODULE_SBUS_GPIO_PIN;
  escBridgeTxGpiox = TRAINER_MODULE_SBUS_GPIO;

  // the trainer port initialization only initializes RX, not TX
  // so TX is assigned AF here
  LL_GPIO_InitTypeDef pinInit;
  LL_GPIO_StructInit(&pinInit);
  pinInit.Pin = escBridgeTxPin;
  pinInit.Mode = LL_GPIO_MODE_ALTERNATE;
  pinInit.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  pinInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  pinInit.Pull = LL_GPIO_PULL_UP;
  pinInit.Alternate = 0x08;//TRAINER_MODULE_SBUS_GPIO_AF;
  // TODO: use the TRAINER_MODULE_SBUS_GPIO_AF definition after fixing compiler errors
  LL_GPIO_Init(escBridgeTxGpiox, &pinInit);
  escBridgeTxPinSetDir(false);
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

int8_t escBridgeAuxInit(uint8_t port_n, const etx_serial_init* params)
{
  #if defined(AUX_SERIAL) || defined(AUX2_SERIAL)
  escBridgePort = (etx_serial_port_t*)serialGetPort(port_n);
  if (!escBridgePort) {
    return ESCBRIDGE_INIT_ERR_NO_PORT;
  }
  escBridgeDrv = (etx_serial_driver_t*)escBridgePort->uart;
  if (!escBridgeDrv) {
    return ESCBRIDGE_INIT_ERR_NO_DRV;
  }
  escBridgeCtx = escBridgeDrv->init(escBridgePort->hw_def, params);
  if (!escBridgeCtx) {
    return ESCBRIDGE_INIT_ERR_NO_CTX;
  }
  #ifdef AUX_SERIAL_GPIO_PIN_TX
  if (port_n == SP_AUX1) {
    escBridgeTxPin = AUX_SERIAL_GPIO_PIN_TX;
    escBridgeTxGpiox = AUX_SERIAL_GPIO;
    escBridgeTxPinSetDir(false);
    return ESCBRIDGE_INIT_SUCCESS;
  }
  #endif
  #ifdef AUX2_SERIAL_GPIO_PIN_TX
  if (port_n == SP_AUX2) {
    escBridgeTxPin = AUX2_SERIAL_GPIO_PIN_TX;
    escBridgeTxGpiox = AUX2_SERIAL_GPIO;
    escBridgeTxPinSetDir(false);
    return ESCBRIDGE_INIT_SUCCESS;
  }
  #endif
  return ESCBRIDGE_INIT_ERR_NO_AUX;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

int8_t escBridgeExtModInit(const etx_serial_init* params)
{
  #if defined(EXTMODULE_USART)
  // this initialization borrows from the existing passthrough code, it is assumed that both TX and RX works
  escBridgeModState = modulePortInitSerial(EXTERNAL_MODULE, ETX_MOD_PORT_UART, params);
  if (!escBridgeModState) {
    return ESCBRIDGE_INIT_ERR_NO_MOD;
  }
  escBridgeDrv = (etx_serial_driver_t*)modulePortGetSerialDrv(escBridgeModState->rx);
  if (!escBridgeDrv) {
    return ESCBRIDGE_INIT_ERR_NO_DRV;
  }
  escBridgeCtx = modulePortGetCtx(escBridgeModState->rx);
  if (!escBridgeCtx) {
    return ESCBRIDGE_INIT_ERR_NO_CTX;
  }
  escBridgeTxPin = EXTMODULE_TX_GPIO_PIN;
  escBridgeTxGpiox = EXTMODULE_USART_GPIO;
  escBridgeTxPinSetDir(false);
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

int8_t escBridgeSportInit(etx_serial_init* params)
{
  #if defined(TELEMETRY_USART)
  escBridgeModState = modulePortInitSerial(1, ETX_MOD_PORT_SPORT, params);
  // note: the S.PORT connector on the bottom of the X7 is not a SPORT_MODULE, it is a part of EXTERNAL_MODULE
  // TODO: use a constant for EXTERNAL_MODULE, after fixing compiler errors
  // TODO: make sure this works for all radios, as some has an inversion selection

  if (!escBridgeModState) {
    return ESCBRIDGE_INIT_ERR_NO_MOD;
  }
  escBridgeDrv = (etx_serial_driver_t*)modulePortGetSerialDrv(escBridgeModState->rx);
  if (!escBridgeDrv) {
    return ESCBRIDGE_INIT_ERR_NO_DRV;
  }
  escBridgeCtx = modulePortGetCtx(escBridgeModState->rx);
  if (!escBridgeCtx) {
    return ESCBRIDGE_INIT_ERR_NO_CTX;
  }
  escBridgeTxPinSetDir(false);
  escBridgeInitFifo(); // this will force manual echo to be used
  // need the UART to be not inverted, use setPolarity implementation if available
  if (escBridgeDrv->setPolarity) {
    auto module = modulePortGetModule(escBridgeModState);
    escBridgeDrv->setPolarity(escBridgeCtx, 0);
    // TODO: check if the polarity here is actually correct
  }
  else {
    #if defined(TELEMETRY_REV_GPIO)
    // it would be really weird if this was available but setPolarity was not
    LL_GPIO_ResetOutputPin(TELEMETRY_REV_GPIO, TELEMETRY_TX_REV_GPIO_PIN | TELEMETRY_RX_REV_GPIO_PIN);
    #endif
  }
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}