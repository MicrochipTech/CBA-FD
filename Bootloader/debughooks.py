# Defines
BOOTLOADER_REQUEST  = 0x424F4F54
SKIP_CHECKSUM       = 0x5500AA00

# V71 Register map
BRAM    = 0x40074000

def on_reset(api,resetAdr):
  api.Print("ABA Bootloader Debug script running!", "Debug")

  # Bootloader request
  api.Write32(BRAM, BOOTLOADER_REQUEST)