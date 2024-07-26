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

#include "stm32_pulse_driver.h"
#include "stm32_serial_driver.h"

#include "hal/trainer_driver.h"
#include "hal/module_port.h"

#include "hal.h"

#include "opentx.h"
#include "trainer.h"

static void (*_trainer_timer_isr)();
static const stm32_pulse_timer_t* _trainer_timer;

void trainer_init()
{
  _trainer_timer = nullptr;
  _trainer_timer_isr = nullptr;
}

static inline bool trainer_check_isr_flag(const stm32_pulse_timer_t* tim)
{
  switch (tim->TIM_Channel) {
    case LL_TIM_CHANNEL_CH1:
      if (LL_TIM_IsEnabledIT_CC1(tim->TIMx) &&
          LL_TIM_IsActiveFlag_CC1(tim->TIMx)) {
        LL_TIM_ClearFlag_CC1(tim->TIMx);
        return true;
      }
      break;
    case LL_TIM_CHANNEL_CH2:
      if (LL_TIM_IsEnabledIT_CC2(tim->TIMx) &&
          LL_TIM_IsActiveFlag_CC2(tim->TIMx)) {
        LL_TIM_ClearFlag_CC2(tim->TIMx);
        return true;
      }
      break;
    case LL_TIM_CHANNEL_CH3:
      if (LL_TIM_IsEnabledIT_CC3(tim->TIMx) &&
          LL_TIM_IsActiveFlag_CC3(tim->TIMx)) {
        LL_TIM_ClearFlag_CC3(tim->TIMx);
        return true;
      }
      break;
    case LL_TIM_CHANNEL_CH4:
      if (LL_TIM_IsEnabledIT_CC4(tim->TIMx) &&
          LL_TIM_IsActiveFlag_CC4(tim->TIMx)) {
        LL_TIM_ClearFlag_CC4(tim->TIMx);
        return true;
      }
      break;
  }
  return false;
}

static inline void capture_pulse(uint16_t capture)
{
  static uint16_t lastCapt = 0;
  static uint8_t channel = MAX_TRAINER_CHANNELS;

  uint16_t val = (uint16_t)(capture - lastCapt) / 2;
  lastCapt = capture;

  if (val > 4000 && val < 19000) {
    // blanking period in [4..19] milliseconds
    channel = 0;
    return;
  }

  if (channel >= MAX_TRAINER_CHANNELS) {
    return;
  }

  if (val < 800 || val > 2200) {
    // invalid pulse width
    channel = MAX_TRAINER_CHANNELS;
    return;
  }
  
  trainerInput[channel++] =
    // +-500 != 512, but close enough.
    (int16_t)(val - 1500) * (g_eeGeneral.PPM_Multiplier + 10) / 10;

  trainerResetTimer();
}

static void trainer_in_isr()
{
  // proceed only if the channel flag was set
  // and the IRQ was enabled
  if (!trainer_check_isr_flag(_trainer_timer))
    return;

  uint16_t capture = 0;
  switch(_trainer_timer->TIM_Channel) {
  case LL_TIM_CHANNEL_CH1:
    capture = LL_TIM_IC_GetCaptureCH1(_trainer_timer->TIMx);
    break;
  case LL_TIM_CHANNEL_CH2:
    capture = LL_TIM_IC_GetCaptureCH2(_trainer_timer->TIMx);
    break;
  case LL_TIM_CHANNEL_CH3:
    capture = LL_TIM_IC_GetCaptureCH3(_trainer_timer->TIMx);
    break;
  case LL_TIM_CHANNEL_CH4:
    capture = LL_TIM_IC_GetCaptureCH4(_trainer_timer->TIMx);
    break;
  default:
    return;
  }

  capture_pulse(capture);
}

void trainer_stop()
{
  stm32_pulse_deinit(_trainer_timer);
  _trainer_timer_isr = nullptr;
  _trainer_timer = nullptr;
}

void trainer_init_capture(const stm32_pulse_timer_t* tim)
{
  // set proper ISR handler first
  _trainer_timer = tim;
  _trainer_timer_isr = trainer_in_isr;

  stm32_pulse_init(tim, 0);
  stm32_pulse_config_input(tim);

  switch (tim->TIM_Channel) {
  case LL_TIM_CHANNEL_CH1:
    LL_TIM_EnableIT_CC1(tim->TIMx);
    break;
  case LL_TIM_CHANNEL_CH2:
    LL_TIM_EnableIT_CC2(tim->TIMx);
    break;
  case LL_TIM_CHANNEL_CH3:
    LL_TIM_EnableIT_CC3(tim->TIMx);
    break;
  case LL_TIM_CHANNEL_CH4:
    LL_TIM_EnableIT_CC4(tim->TIMx);
    break;
  }

  LL_TIM_EnableCounter(tim->TIMx);
}

static void trainer_send_next_frame(const stm32_pulse_timer_t* tim)
{
  stm32_pulse_set_polarity(tim, GET_TRAINER_PPM_POLARITY());
  stm32_pulse_set_cmp_val(tim, GET_TRAINER_PPM_DELAY() * 2);
  
  // load the first period: next reload when CC compare event triggers
  trainerPulsesData.ppm.ptr = trainerPulsesData.ppm.pulses;
  tim->TIMx->ARR = *(trainerPulsesData.ppm.ptr++);

  switch (tim->TIM_Channel) {
  case LL_TIM_CHANNEL_CH1:
    LL_TIM_EnableIT_CC1(tim->TIMx);
    break;
  case LL_TIM_CHANNEL_CH2:
    LL_TIM_EnableIT_CC2(tim->TIMx);
    break;
  case LL_TIM_CHANNEL_CH3:
    LL_TIM_EnableIT_CC3(tim->TIMx);
    break;
  case LL_TIM_CHANNEL_CH4:
    LL_TIM_EnableIT_CC4(tim->TIMx);
    break;
  }
}

static void trainer_out_isr()
{
  // proceed only if the channel flag was set
  // and the IRQ was enabled
  if (!trainer_check_isr_flag(_trainer_timer))
    return;

  if (*trainerPulsesData.ppm.ptr) {
    // load next period
    LL_TIM_SetAutoReload(_trainer_timer->TIMx,
                         *(trainerPulsesData.ppm.ptr++));
  } else {
    setupPulsesPPMTrainer();
    trainer_send_next_frame(_trainer_timer);
  }  
}

void trainer_init_output(const stm32_pulse_timer_t* tim)
{
  // set proper ISR handler first
  _trainer_timer = tim;
  _trainer_timer_isr = trainer_out_isr;

  stm32_pulse_init(_trainer_timer, 0);
  stm32_pulse_config_output(_trainer_timer, GET_TRAINER_PPM_POLARITY(),
                            LL_TIM_OCMODE_PWM1, GET_TRAINER_PPM_DELAY() * 2);

  setupPulsesPPMTrainer();
  trainer_send_next_frame(_trainer_timer);

  LL_TIM_EnableCounter(_trainer_timer->TIMx);
}

void trainer_timer_isr()
{
  if (_trainer_timer && _trainer_timer_isr)
    _trainer_timer_isr();
}

#if defined(TRAINER_MODULE_CPPM)

#include "hal/module_port.h"

static_assert(__IS_TRAINER_TIMER_IN_CHANNEL_SUPPORTED(TRAINER_MODULE_CPPM_TIMER_Channel),
              "Unsupported trainer timer input channel");

static const stm32_pulse_timer_t trainerModuleTimer = {
  .GPIOx = TRAINER_MODULE_CPPM_GPIO,
  .GPIO_Pin = TRAINER_MODULE_CPPM_GPIO_PIN,
  .GPIO_Alternate = TRAINER_MODULE_CPPM_GPIO_AF,
  .TIMx = TRAINER_MODULE_CPPM_TIMER,
  .TIM_Freq = TRAINER_MODULE_CPPM_FREQ,
  .TIM_Channel = TRAINER_MODULE_CPPM_TIMER_Channel,
  .TIM_IRQn = TRAINER_MODULE_CPPM_TIMER_IRQn,
  .DMAx = nullptr,
  .DMA_Stream = 0,
  .DMA_Channel = 0,
  .DMA_IRQn = (IRQn_Type)0,
  .DMA_TC_CallbackPtr = nullptr,
};

void init_trainer_module_cppm()
{
  modulePortSetPower(EXTERNAL_MODULE,true);
  _init_trainer_capture(&trainerModuleTimer);
}

void stop_trainer_module_cppm()
{
  _stop_trainer(&trainerModuleTimer);
  modulePortSetPower(EXTERNAL_MODULE,false);
}

#if defined(TRAINER_MODULE_CPPM_TIMER_IRQHandler)
extern "C" void TRAINER_MODULE_CPPM_TIMER_IRQHandler()
{
  DEBUG_INTERRUPT(INT_TRAINER);

  if (_trainer_timer && _trainer_timer_isr)
    _trainer_timer_isr();
}
#endif

#endif // TRAINER_MODULE_CPPM

#if defined(TRAINER_MODULE_SBUS)

const etx_serial_init sbusTrainerParams = {
    .baudrate = SBUS_BAUDRATE,
    .encoding = ETX_Encoding_8E2,
    .direction = ETX_Dir_RX,
    .polarity = ETX_Pol_Normal,
};

// external module may have a full-duplex USART
#if defined(EXTMODULE_USART) || defined(CONFIGURABLE_MODULE_PORT)

#include "hal/module_port.h"

static etx_module_state_t* sbus_trainer_mod_st = nullptr;

void init_trainer_module_sbus()
{
  if (sbus_trainer_mod_st) return;
  modulePortSetPower(EXTERNAL_MODULE,true);

  sbus_trainer_mod_st = modulePortInitSerial(EXTERNAL_MODULE, ETX_MOD_PORT_UART,
                                             &sbusTrainerParams, false);
}

void stop_trainer_module_sbus()
{
  if (!sbus_trainer_mod_st) return;
  modulePortDeInit(sbus_trainer_mod_st);
  modulePortSetPower(EXTERNAL_MODULE,false);
  sbus_trainer_mod_st = nullptr;
}

int trainerModuleSbusGetByte(uint8_t* data)
{
  if (!sbus_trainer_mod_st) return 0;

  auto serial_driver = modulePortGetSerialDrv(sbus_trainer_mod_st->rx);
  auto ctx = modulePortGetCtx(sbus_trainer_mod_st->rx);

  if (ctx) {
    return serial_driver->getByte(ctx, data);
  }

  return 0;
}

#elif defined(TRAINER_MODULE_SBUS_USART)
#include "stm32_serial_driver.h"

static const stm32_usart_t sbus_trainer_USART = {
  .USARTx = TRAINER_MODULE_SBUS_USART,
  .GPIOx = TRAINER_MODULE_SBUS_GPIO,
  .GPIO_Pin = TRAINER_MODULE_SBUS_GPIO_PIN,
  .IRQn = (IRQn_Type)-1,
  .IRQ_Prio = 0,
  .txDMA = nullptr,
  .txDMA_Stream = 0,
  .txDMA_Channel = 0,
  .rxDMA = TRAINER_MODULE_SBUS_DMA,
  .rxDMA_Stream = TRAINER_MODULE_SBUS_DMA_STREAM_LL,
  .rxDMA_Channel = TRAINER_MODULE_SBUS_DMA_CHANNEL,
};

DEFINE_STM32_SERIAL_PORT(SbusTrainer, sbus_trainer_USART, 32, 0);

static void* _sbus_trainer_ctx = nullptr;

void init_trainer_module_sbus()
{
  _sbus_trainer_ctx = STM32SerialDriver.init(REF_STM32_SERIAL_PORT(SbusTrainer),
                                             &sbusTrainerParams);
}

void stop_trainer_module_sbus()
{
  STM32SerialDriver.deinit(_sbus_trainer_ctx);
  _sbus_trainer_ctx = nullptr;
}

int trainerModuleSbusGetByte(uint8_t* data)
{
  return STM32SerialDriver.getByte(_sbus_trainer_ctx, data);
}

void* trainerGetContext()
{
  return _sbus_trainer_ctx;
}

#else
  #error "No available SBUS trainer implementation"
#endif
#endif // TRAINER_MODULE_SBUS