/**
 * \file
 *
 * \brief GCC linker script (flash) for ATSAMV71Q21B
 *
 * Copyright (c) 2017 Atmel Corporation, a wholly owned subsidiary of Microchip Technology Inc.
 *
 * \license_start
 *
 * \page License
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \license_stop
 *
 */

/*------------------------------------------------------------------------------
 *      Linker script for running in internal FLASH on the ATSAMV71Q21B
 *----------------------------------------------------------------------------*/

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

/* Memory Spaces Definitions */
MEMORY
{
	itcm (rwx)   : ORIGIN = 0x00000000, LENGTH = 0x00010000
  bldr (rx)    : ORIGIN = 0x00400000, LENGTH = 0x00020000
	appl (rx)    : ORIGIN = 0x00420000, LENGTH = 0x001DE000
	appc (r)     : ORIGIN = 0x005FF000, LENGTH = 0x00001000
	dtcm (rw)    : ORIGIN = 0x20000000, LENGTH = 0x00010000
	ram (rwx)    : ORIGIN = 0x20400000, LENGTH = 0x00034000
  ramnc (rwx)  : ORIGIN = 0x20434000, LENGTH = 0x0000C000
}

HEAP_SIZE  = 0x0;
STACK_SIZE = 0x1000;

INCLUDE samv71_flash.ld

_ram_start_         = ORIGIN(ram  );
_ram_end_           = ORIGIN(ram  ) + LENGTH(ram  ) - 1;
_ram_nocache_start_ = ORIGIN(ramnc);
_ram_nocache_end_   = ORIGIN(ramnc) + LENGTH(ramnc) - 1;

_appl_start_  = ORIGIN(appl);
_appl_end_    = _appl_start_ + LENGTH(appl) - 1;
_dtcm_start_  = ORIGIN(dtcm);
_dtcm_end_    = _dtcm_start_ + LENGTH(dtcm) - 1;