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
 * (it helps if Bluetooth is disabled)
 * 
 * To use this feature:
 * Select CLI as the USB-VCP's mode (via the hardware menu)
 * Plug in the USB cable, select VCP mode (when prompted on the screen)
 * Open a terminal app on the host PC, type in the command
 *     serialpassthrough escbridge <port> <baud>
 * such as
 *     serialpassthrough escbridge aux1 19200
 * Then close the terminal app and use the ESC's dedicated PC app
*/

#include "hal.h" // for most of the board definitions
#include "stm32_hal_ll.h"
#include "dataconstants.h"
#include "myeeprom.h"
#include "serial.h"
#include "hal/module_port.h"
#include "stm32_softserial_driver.h"

#include "esc_bridge.h"

#include <stdlib.h>

//#define ESCBRIDGE_USE_HALFDUPLEX    // using native half-duplex mode, which means echo is required and some other quirks

#if defined(TRAINER_MODULE_SBUS_USART)
void* trainerGetContext();
void init_trainer_module_sbus();
#endif

extern const etx_serial_driver_t STM32SerialDriver;
extern const etx_serial_driver_t STM32SoftSerialTxDriver;
extern const etx_serial_driver_t STM32SoftSerialRxDriver;

static etx_serial_port_t* escBridgePort = nullptr;
static etx_serial_driver_t* escBridgeDrv = nullptr;
static etx_module_state_t* escBridgeModState = nullptr;
static void* escBridgeCtx = nullptr;
static etx_serial_port_t* escBridgePortRx = nullptr;
static etx_serial_driver_t* escBridgeDrvRx = nullptr;
static etx_module_state_t* escBridgeModStateRx = nullptr;
static void* escBridgeCtxRx = nullptr;
static uint32_t escBridgeTxPin = 0;
static USART_TypeDef* escBridgeTxUsartx = nullptr;
static GPIO_TypeDef* escBridgeTxGpiox = nullptr;
// TODO: using raw STM32 items here makes this code not portable to other microcontroller families

static bool escBridgeHasDirPin = false;       // indicates that direction change will happen automatically
static bool escBridgeUsingHalfDuplex = false; // using native half-duplex mode, which means echo is required and some other quirks
static bool escBridgeRequireEcho = false;     // implementations of half-duplex will disable RX while TX, so a manual echo is needed
static bool escBridgeNoPinModes = false;      // the JR bay implementation does not require any action for direction change

// if manual echo is required, then implement a FIFO to store all the bytes to echo out (must be power of 2)
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
  if (escBridgeNoPinModes) {
    return;
  }
  if (isOut == false) { // RX mode
    if (escBridgeTxGpiox && escBridgeUsingHalfDuplex) {
      LL_GPIO_SetPinPull(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_PULL_UP);
    }
    if (escBridgeHasDirPin == false && escBridgeUsingHalfDuplex == false) {
      LL_GPIO_SetPinMode(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_MODE_INPUT);
    }
    escBridgeDrv->enableRx(escBridgeCtx);
    if (escBridgeUsingHalfDuplex) {
      LL_USART_DisableDirectionTx(escBridgeTxUsartx);
    }
  }
  else { // TX mode
    if (escBridgeTxGpiox && escBridgeUsingHalfDuplex) {
      LL_GPIO_SetPinPull(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_PULL_NO);
    }
    LL_USART_EnableDirectionTx(escBridgeTxUsartx);
    LL_USART_DisableDirectionRx(escBridgeTxUsartx);
    if (escBridgeHasDirPin == false && escBridgeUsingHalfDuplex == false) {
      LL_GPIO_SetPinMode(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_MODE_ALTERNATE);
    }
  }
}

static void escBridgeCfgUart(USART_TypeDef* usart, GPIO_TypeDef* gpio, uint32_t pin)
{
  escBridgeTxUsartx = usart;
  LL_USART_SetStopBitsLength(escBridgeTxUsartx, LL_USART_STOPBITS_1); 
  // TODO: what other things do we need to make sure of?
  #ifdef ESCBRIDGE_USE_HALFDUPLEX
  LL_USART_EnableHalfDuplex(escBridgeTxUsartx);
  escBridgeUsingHalfDuplex = true;
  #else
  LL_USART_DisableHalfDuplex(escBridgeTxUsartx);
  escBridgeUsingHalfDuplex = false;
  #endif

  escBridgeTxGpiox = gpio;
  escBridgeTxPin = pin;
  #ifdef ESCBRIDGE_USE_HALFDUPLEX
  // TODO: make sure all of this is needed
  // it seemed like half-duplex mode is not working right, and my research pointed to perhaps more strict GPIO configuration
  // but it turns out, my X9D simply had a inverted AUX TX signal, which made me think something was wrong with the pin
  LL_GPIO_SetPinMode(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(escBridgeTxGpiox, escBridgeTxPin, 
    #ifdef LL_GPIO_SPEED_FREQ_VERY_HIGH
    LL_GPIO_SPEED_FREQ_VERY_HIGH
    #elif defined(LL_GPIO_SPEED_FREQ_HIGH)
    LL_GPIO_SPEED_FREQ_HIGH
    #endif
    );
  LL_GPIO_SetPinPull(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_PULL_NO);
  #else
  LL_GPIO_SetPinPull(escBridgeTxGpiox, escBridgeTxPin, LL_GPIO_PULL_UP);
  #endif
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
  // need to wait until all the bytes are sent physically, then the TX pin must be turned into an input
  //escBridgeDrv->waitForTxCompleted(escBridgeCtx);
  // waitForTxCompleted seems to be not effective, on_idle callback is also ineffective, both of those are waiting for TXE, we need to wait for TC flag
  if (escBridgeTxUsartx != nullptr) {
    while ((escBridgeTxUsartx->SR & USART_SR_TC) == 0) {
      // do nothing but wait
    }
  }
  // note: the driver for software TX is already always blocking
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

  if (escBridgeDrvRx != nullptr && escBridgeDrvRx->getByte != nullptr) {
    if (escBridgeDrvRx->getByte(escBridgeCtxRx, &data) > 0) { // yes data
      return (int16_t)data;
    }
  }
  else if (escBridgeDrv != nullptr && escBridgeDrv->getByte != nullptr) {
    if (escBridgeDrv->getByte(escBridgeCtx, &data) > 0) { // yes data
      return (int16_t)data;
    }
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
  escBridgeCfgUart(TRAINER_MODULE_SBUS_USART, TRAINER_MODULE_SBUS_GPIO, TRAINER_MODULE_SBUS_GPIO_PIN);
  // TODO: I don't think the pin here is correct, it's mapped to RX, not TX
  // the TX pin might not actually exist
  escBridgeInitFifo();

  // the trainer port initialization only initializes RX, not TX
  // so TX is assigned AF here
  LL_GPIO_InitTypeDef pinInit;
  LL_GPIO_StructInit(&pinInit);
  pinInit.Pin = escBridgeTxPin;
  pinInit.Mode = LL_GPIO_MODE_ALTERNATE;
  pinInit.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  pinInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  pinInit.Pull = LL_GPIO_PULL_UP;
  pinInit.Alternate = TRAINER_MODULE_SBUS_GPIO_AF;
  LL_GPIO_Init(escBridgeTxGpiox, &pinInit);
  escBridgeTxPinSetDir(false);
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

int8_t escBridgeAuxInit(uint8_t port_n, const etx_serial_init* params)
{
  UNUSED(escBridgePort);
  UNUSED(escBridgePortRx);
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
    escBridgeCfgUart(AUX_SERIAL_USART, AUX_SERIAL_GPIO, AUX_SERIAL_GPIO_PIN_TX);
    escBridgeInitFifo();
    escBridgeTxPinSetDir(false);
    return ESCBRIDGE_INIT_SUCCESS;
  }
  #endif
  #ifdef AUX2_SERIAL_GPIO_PIN_TX
  if (port_n == SP_AUX2) {
    escBridgeCfgUart(AUX2_SERIAL_USART, AUX2_SERIAL_GPIO, AUX2_SERIAL_GPIO_PIN_TX);
    escBridgeInitFifo();
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
  escBridgeModState = modulePortInitSerial(EXTERNAL_MODULE, ETX_MOD_PORT_UART, params, false);
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
  escBridgeCfgUart(EXTMODULE_USART, EXTMODULE_USART_GPIO, EXTMODULE_TX_GPIO_PIN);
  escBridgeInitFifo();
  escBridgeTxPinSetDir(false);
  modulePortSetPower(EXTERNAL_MODULE, true);
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

int8_t escBridgeSportInit(etx_serial_init* params)
{
  #if defined(TELEMETRY_USART)
  escBridgeModState = modulePortInitSerial(EXTERNAL_MODULE, ETX_MOD_PORT_SPORT, params, false);
  // note: the S.PORT connector on the bottom of the X7 is not a SPORT_MODULE, it is a part of EXTERNAL_MODULE
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
  escBridgeTxUsartx = TELEMETRY_USART;
  escBridgeHasDirPin = true;
  escBridgeTxPinSetDir(false);
  escBridgeInitFifo(); // this will force manual echo to be used
  // need the UART to be not inverted, use setPolarity implementation if available
  if (escBridgeDrv->setPolarity) {
    escBridgeDrv->setPolarity(escBridgeCtx, 0);
    // TODO: check if the polarity here is actually correct
  }
  else {
    #if defined(TELEMETRY_REV_GPIO)
    // it would be really weird if this was available but setPolarity was not
    LL_GPIO_ResetOutputPin(TELEMETRY_REV_GPIO, TELEMETRY_TX_REV_GPIO_PIN | TELEMETRY_RX_REV_GPIO_PIN);
    #endif
  }
  // TODO: if the UART cannot have the correct polarity, perhaps notify the user with an error
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

int8_t escBridgeJrBayInit(etx_serial_init* params)
{
  // this is a full duplex port, but the TX pin is open drain, at least according to the schematic
  // so the TX and RX pins can be tied together if being used for an ESC
  #if defined(TRAINER_MODULE_CPPM_TIMER) && defined(INTMODULE_HEARTBEAT_GPIO_PIN) && defined(HARDWARE_EXTERNAL_MODULE) && defined(EXTMODULE_TIMER)
  pulsesStop(); // deinit required or else the TX won't work

  etx_serial_init temp_params;
  memcpy(&temp_params, params, sizeof(etx_serial_init));
  temp_params.direction = ETX_MOD_DIR_TX; // search for the TX port
  escBridgeModState = modulePortInitSerial(EXTERNAL_MODULE, ETX_MOD_PORT_SOFT_INV, (const etx_serial_init*)&temp_params, false);
  if (!escBridgeModState) {
    return ESCBRIDGE_INIT_ERR_NO_MOD;
  }
  escBridgeDrv   = (etx_serial_driver_t*)&STM32SoftSerialTxDriver;
  escBridgeDrvRx = (etx_serial_driver_t*)&STM32SoftSerialRxDriver;
  void* hw_def = nullptr;
  if (escBridgeModState->tx.port) {
    hw_def = escBridgeModState->tx.port->hw_def;
  }
  if (!hw_def && escBridgeModState->rx.port) {
    hw_def = escBridgeModState->rx.port->hw_def;
  }
  if (!hw_def) {
    return ESCBRIDGE_INIT_ERR_NO_DRV;
  }
  escBridgeCtx = hw_def;
  if (escBridgeDrv->setPolarity) {
    escBridgeDrv->setPolarity(escBridgeCtx, true); // this sets the polarity of the PPM generator
  }

 // finished TX initialization, now initialize RX

  #if 0
  // TODO: this section of code that uses HEARTBEAT doesn't work
  // the initialization succeeds, but it only receives maybe a null and then never a complete byte
  // I am unsure if the problem is firmware or hardware
  static stm32_softserial_rx_port softRx = {
    .GPIOx = INTMODULE_HEARTBEAT_GPIO,
    .GPIO_Pin = INTMODULE_HEARTBEAT_GPIO_PIN,
    .TIMx = TRAINER_MODULE_CPPM_TIMER,
    .TIM_Freq = TRAINER_MODULE_CPPM_FREQ,
    .TIM_IRQn = TRAINER_MODULE_CPPM_TIMER_IRQn,
    .EXTI_Port = INTMODULE_HEARTBEAT_EXTI_PORT,
    .EXTI_SysLine = INTMODULE_HEARTBEAT_EXTI_SYS_LINE,
    .EXTI_Line = INTMODULE_HEARTBEAT_EXTI_LINE,
    .dir_GPIOx = nullptr,
    .dir_Pin = 0,
    .dir_Input = 0,
    .buffer = { (uint8_t*)malloc(ESCBRIDGE_FIFO_SIZE), ESCBRIDGE_FIFO_SIZE },
  };
  hw_def = (void*)&softRx;
  temp_params.polarity = ETX_Pol_Normal;
  escBridgeCtxRx = escBridgeDrvRx->init(hw_def, (const etx_serial_init*)&temp_params);
  #else
  // TODO: this section of code uses the SPORT pin in a bit-bang mode
  temp_params.polarity  = ETX_Pol_Normal; // setPolarity is missing, NORMAL makes the pin uninverted
  temp_params.direction = ETX_MOD_DIR_RX; // search for the RX port
  escBridgeModStateRx = modulePortInitSerial(EXTERNAL_MODULE, ETX_MOD_PORT_SPORT_INV, (const etx_serial_init*)&temp_params, false);
  if (!escBridgeModStateRx) {
    return ESCBRIDGE_INIT_ERR_NO_MOD + ESCBRIDGE_INIT_ERR_SECOND;
  }
  if (escBridgeModStateRx->rx.port) {
    hw_def = escBridgeModStateRx->rx.port->hw_def;
  }
  escBridgeCtxRx = hw_def;
  #endif
  if (!escBridgeCtxRx) {
    return ESCBRIDGE_INIT_ERR_NO_CTX + ESCBRIDGE_INIT_ERR_SECOND;
  }
  if (escBridgeDrvRx->setPolarity) {
    escBridgeDrvRx->setPolarity(escBridgeCtxRx, true); // TODO: verify this works right (the RX driver has no such function right now)
  }
  escBridgeNoPinModes = true; // the PPM pin is open drain, and it's not using an USART peripheral, so we don't need to ever toggle TX pin modes
  modulePortSetPower(EXTERNAL_MODULE, true);
  return ESCBRIDGE_INIT_SUCCESS;
  #endif
  return ESCBRIDGE_INIT_ERR_NO_HW;
}

#include "lcd.h"

void escBridgeDisplayLcd()
{
  lcdRefreshWait();
  lcdClear();
  lcdDrawCenteredText((LCD_H - FH) / 2, "ESC Bridge Active");
  lcdRefresh();
  lcdRefreshWait();
}