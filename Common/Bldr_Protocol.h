/*
 * Bldr_Protocol.h
 *
 * Created: 11.10.2017 10:50:31
 *  Author: M43734
 */ 

#pragma once

#include <stdint.h>

#define BRAM_MAGIC          (0x12345678U)
#define BOOTLOADER_REQUEST  (0x424F4F54U)
#define SAMBA_REQUEST       (0x54545454U)
#define SKIP_CHECKSUM       (0x5500AA00U)

#define MAGIC_ACTION        (0xAC7104E0U)
#define MAGIC_REPLY         (0x52E54C59U)

#define BOOTLDR_VERSION_1   (0x00000001U)
#define BOOTLDR_VERSION_2   (0x00000002U)

enum eBootsettings
{
  BOOT_BOOTLOADER_REQ  = 0x0u,
  BOOT_BOOTLOADER_REQN = 0x1u,
};

enum eAction
{
  NOP = 0,
  ERASESTART,
  ERASEFLASH,
  ERASESTATUS,
  SENDDATA,
  WRITEDATA,
  QUERYVERSION,
  RESET,
  ERROR_OK,
  ERROR_CHECKSUM,
  TWINKLE
};

COMPILER_PACK_SET(1)
struct sbAction
{
  uint32_t magic;
  uint32_t action;
  uint32_t checksum;
  uint32_t address;
  uint8_t data[256];
};

COMPILER_PACK_RESET()

COMPILER_PACK_SET(4)
struct sbReply
{
  uint32_t magic;
  uint32_t action;
  uint32_t reply;
  uint32_t reserved;
};

struct bRamLayout
{
  volatile uint32_t bootloader_req;
  volatile uint32_t skip_checksum;
  volatile uint32_t bootloader_ver;
  volatile uint32_t board_ver;
};
COMPILER_PACK_RESET()
