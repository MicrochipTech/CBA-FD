/*
 * settings.c
 *
 * Created: 14.05.2019 09:18:58
 *  Author: M43734
 */ 

#include <board.h>
#include <stdint.h>
#include <stdio.h>

#define JSMN_PARENT_LINKS
#include <jsmn.h>
#include <settings.h>
#include "tasks/task_CAN_cfg.h"

#define settings_printf printf

void inputTrigger(void)
{
  settings_printf("-- InputTrigger\r\n");
}

static sdCanCfg_t sdCanCfg;
static gpioConfig_t gpioConfig;
static sdCfg_t sdCfg;

static inline int getTokenLength(jsmntok_t* t)
{
  return t->end - t->start;
}

static bool jsoneq(const char *jsn, jsmntok_t *t, const char *s)
{
  if (t->type == JSMN_STRING && (int)strlen(s) == getTokenLength(t))
  {
    if (strncmp(jsn + t->start, s, getTokenLength(t)) == 0)
    {
      return true;
    }
  }
  return false;
}

static int parseGpio(const char* jsn, jsmntok_t* t, gpio_t* gpio)
{
  /**
  JSON values always come in pairs. E.g. STRING:OBJECT, OBJECT:OBJECT, STRING:STRING, except for arrays which have a 1:n mapping.
  
  ---- SNIP JSON ----
  0:  "GPIOS":            STRING
  1:  {                 OBJECT    (size = 1 + n)
  2:  "T1":             STRING
  3:  {               OBJECT    (size = 2)  <= this object is passed to the function
  4:    "dir": "input",       STRING:STRING
  5:    "action": "invalidtest"   STRING:STRING
  6:  },
  7:  ...
	---- SNIP JSON ----
  **/

  int i = 1;

  // First node request must be of type object
  if (t[i].type == JSMN_OBJECT)
  {
    // Retrieve the number of child nodes
    int size = t[i].size;
    i++;

    // Store the id of the top-level node
    int myParent = t[i].parent;

    // Enumerate through all child nodes
    while (size--)
    {
      if (jsoneq(jsn, &t[i], "dir"))
      {
        i++;

        if (jsoneq(jsn, &t[i], "output"))
        {
          settings_printf("Direction: Output\r\n");
          gpio->direction = 1;
        }
        else
        {
          settings_printf("Direction: Input\r\n");
          gpio->direction = 0;
        }
      }
      else if (jsoneq(jsn, &t[i], "action"))
      {
        i++;

        if (jsoneq(jsn, &t[i], "inputtrigger"))
        {
          settings_printf("Function: InputTrigger\r\n");
          gpio->func_ptr = (uintptr_t)inputTrigger;
        }
      }
      else
      {
        i++;
      }
			
      // Skip ahead to the next child node. If parent is smaller than myParent we have passed the last child.
      while(t[i].parent > myParent)
        i++;
    }
  }

  return i;
}

static int parseGpios(const char* jsn, jsmntok_t* t)
{
  int i = 1;
  if (t[i].type == JSMN_OBJECT)
  {
    // Retrieve the number of child nodes
    int size = t[i].size;
    i++;

    // Store the id of the top-level node
    int myParent = t[i].parent;

    while (size--)
    {
      if (jsoneq(jsn, &t[i], "T1"))
      {
        settings_printf("Config of T1:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.trigger[0]);
      }
      else if (jsoneq(jsn, &t[i], "T2"))
      {
        settings_printf("Config of T2:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.trigger[1]);
      }
      else if (jsoneq(jsn, &t[i], "GPIO0"))
      {
        settings_printf("Config of GPIO0:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.gpio[0]);
      }
      else if (jsoneq(jsn, &t[i], "GPIO1"))
      {
        settings_printf("Config of GPIO1:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.gpio[1]);
      }
      else if (jsoneq(jsn, &t[i], "GPIO2"))
      {
        settings_printf("Config of GPIO2:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.gpio[2]);
      }
      else if (jsoneq(jsn, &t[i], "GPIO3"))
      {
        settings_printf("Config of GPIO3:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.gpio[3]);
      }
      else if (jsoneq(jsn, &t[i], "GPIO4"))
      {
        settings_printf("Config of GPIO4:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.gpio[4]);
      }
      else if (jsoneq(jsn, &t[i], "GPIO5"))
      {
        settings_printf("Config of GPIO5:\r\n");
        i += parseGpio(jsn, &t[i], &gpioConfig.gpio[5]);
      }
      else
      {
        i++;
      }

      while (t[i].parent > myParent)
        i++;
    }
  }

  return i;
}

static int parseCan(const char* jsn, jsmntok_t* t, uint8_t devId)
{
  int i = 1;

  if (t[i].type == JSMN_OBJECT)
  {
    // Retreive the number of child nodes
    int size = t[i].size;
    i++;

    // Store the id of the top-level node
    int myParent = t[i].parent;

    while(size--)
    {
      if (jsoneq(jsn, &t[i], "nseg1"))
      {
        i++;
        if (t[i].type == JSMN_PRIMITIVE)
        {
          sdCanCfg.sdInitCanSpeed[devId].nseg1 = strtoul(jsn + t[i].start, 0, 10);
        }
      }
      else if (jsoneq(jsn, &t[i], "nseg2"))
      {
        i++;
        if (t[i].type == JSMN_PRIMITIVE)
        {
          sdCanCfg.sdInitCanSpeed[devId].nseg2 = strtoul(jsn + t[i].start, 0, 10);
          sdCanCfg.sdInitCanSpeed[devId].nsjw = sdCanCfg.sdInitCanSpeed[devId].nseg2;
        }
      }
      else if (jsoneq(jsn, &t[i], "dseg1"))
      {
        i++;
        if (t[i].type == JSMN_PRIMITIVE)
        {
          sdCanCfg.sdInitCanSpeed[devId].dseg1 = strtoul(jsn + t[i].start, 0, 10);
        }
      }
      else if (jsoneq(jsn, &t[i], "dseg2"))
      {
        i++;
        if (t[i].type == JSMN_PRIMITIVE)
        {
          sdCanCfg.sdInitCanSpeed[devId].dseg2 = strtoul(jsn + t[i].start, 0, 10);
          sdCanCfg.sdInitCanSpeed[devId].dsjw = sdCanCfg.sdInitCanSpeed[devId].dseg2;
        }
      }
      else if (jsoneq(jsn, &t[i], "tdc"))
      {
        i++;
        if (t[i].type == JSMN_PRIMITIVE)
        {
          sdCanCfg.sdInitCanSpeed[devId].tdc = strtoul(jsn + t[i].start, 0, 10);
        }
      }
      else if (jsoneq(jsn, &t[i], "mode"))
      {
        i++;
        
        if (jsoneq(jsn, &t[i], "classic"))
        {
          sdCanCfg.sdInitCanMode[devId].mode = aba_can_eCanMode_MODE_CLASSIC;
        }
        else if (jsoneq(jsn, &t[i], "fdniso"))
        {
          sdCanCfg.sdInitCanMode[devId].mode = aba_can_eCanMode_MODE_FDNISO;
        }
        else if (jsoneq(jsn, &t[i], "fdiso"))
        {
          sdCanCfg.sdInitCanMode[devId].mode = aba_can_eCanMode_MODE_FDISO;
        }
      }
      else if (jsoneq(jsn, &t[i], "loop"))
      {
        i++;

        if (jsoneq(jsn, &t[i], "normal"))
        {
          sdCanCfg.sdInitCanMode[devId].specialMode = aba_can_eCanSpecialMode_MODE_NORMAL;
        }
        else if (jsoneq(jsn, &t[i], "listen"))
        {
          sdCanCfg.sdInitCanMode[devId].specialMode = aba_can_eCanSpecialMode_MODE_LISTEN;
        }
        else if (jsoneq(jsn, &t[i], "loopintern"))
        {
          sdCanCfg.sdInitCanMode[devId].specialMode = aba_can_eCanSpecialMode_MODE_LOOPBACK_INT;
        }
        else if (jsoneq(jsn, &t[i], "loopextern"))
        {
          sdCanCfg.sdInitCanMode[devId].specialMode = aba_can_eCanSpecialMode_MODE_LOOPBACK_EXT;
        }
      }
      else
      {
        i++;
      }

      // Skip ahead to the next child node. If parent is smaller than myParent we have passed the last child.
      while (t[i].parent > myParent)
        i++;
    }
  }

  return i;
}

sdCfg_t* parseSettings(const char* jsn, size_t len)
{
  jsmn_parser p;
  jsmntok_t t[128];
  memset(t, 0, sizeof(t));
  memset(&sdCanCfg, 0, sizeof(sdCanCfg));

  sdCanCfg.sdInitCanMode[0].device  = aba_can_eCanDevice_MCAN0;
  sdCanCfg.sdInitCanSpeed[0].device = aba_can_eCanDevice_MCAN0;
  sdCanCfg.sdInitCanSpeed[0].dscale = 1;
  sdCanCfg.sdInitCanSpeed[0].nscale = 1;

  sdCanCfg.sdInitCanMode[1].device  = aba_can_eCanDevice_MCAN1;
  sdCanCfg.sdInitCanSpeed[0].device = aba_can_eCanDevice_MCAN1;
  sdCanCfg.sdInitCanSpeed[1].dscale = 1;
  sdCanCfg.sdInitCanSpeed[1].nscale = 1;

  jsmn_init(&p);
  int r = jsmn_parse(&p, jsn, len, t, sizeof(t) / sizeof(*t) - 1);
  if (r > 0)
  {
    int i = 1;
    for (int c = 0; c < t->size; c++)
    {
      if (jsoneq(jsn, &t[i], "CAN0"))
      {
        i += parseCan(jsn, &t[i], 0);
      }
      else if (jsoneq(jsn, &t[i], "CAN1"))
      {
        i += parseCan(jsn, &t[i], 1);
      }
      else if (jsoneq(jsn, &t[i], "GPIOS"))
      {
        i += parseGpios(jsn, &t[i]);
      }
      else
      {
        i++;
      }
  
      while (t[i].parent != 0)
      {
        i++;
  
        if (i > r) {
          break;
        }
      }
    }
    sdCfg.initialized = 1;
    sdCfg.canCfg = (uintptr_t)&sdCanCfg;
    sdCfg.gpioCfg = gpioConfig;
  }

  return &sdCfg;
}

sdCfg_t* getSettings()
{
  return &sdCfg;
}