/*
 * settings.h
 *
 * Created: 14.05.2019 09:19:33
 *  Author: M43734
 */ 

#pragma once

typedef struct
{
  uint32_t direction;
  uintptr_t func_ptr;
} gpio_t;

typedef struct
{
  gpio_t gpio[6];
  gpio_t trigger[2];
} gpioConfig_t;

typedef struct
{
  uint32_t  initialized;
  uint32_t  logging;
  uintptr_t canCfg;
  gpioConfig_t gpioCfg;
} sdCfg_t;

sdCfg_t* parseSettings(const char* jsn, size_t len);
sdCfg_t* getSettings();
