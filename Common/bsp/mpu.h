/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

#ifndef _MPU_H_
#define _MPU_H_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/
#define ARM_MODE_USR            0x10

#define PRIVILEGE_MODE 0
#define USER_MODE      1

enum MPU_REGIONS
{
  MPU_DEFAULT_ITCM_REGION = 0,
  MPU_DEFAULT_IFLASH_REGION,
  MPU_DEFAULT_DTCM_REGION,
  MPU_DEFAULT_SRAM_REGION,
  MPU_NOCACHE_SRAM_REGION,
  MPU_PERIPHERALS_REGION,
  MPU_EXT_EBI_REGION,
  MPU_DEFAULT_SDRAM_REGION,
  MPU_QSPIMEM_REGION,
  MPU_USBHSRAM_REGION,
  MPU_REGION_MAX = 16
};

#define MPU_REGION_VALID                        (0x10)
#define MPU_REGION_ENABLE                       (0x01)
#define MPU_REGION_DISABLE                      (0x0)

#define MPU_ENABLE                              (0x1 << MPU_CTRL_ENABLE_Pos)
#define MPU_HFNMIENA                            (0x1 << MPU_CTRL_HFNMIENA_Pos)
#define MPU_PRIVDEFENA                          (0x1 << MPU_CTRL_PRIVDEFENA_Pos)


#define MPU_REGION_BUFFERABLE                   (0x01 << MPU_RASR_B_Pos)
#define MPU_REGION_CACHEABLE                    (0x01 << MPU_RASR_C_Pos)
#define MPU_REGION_SHAREABLE                    (0x01 << MPU_RASR_S_Pos)

#define MPU_REGION_EXECUTE_NEVER                (0x01 << MPU_RASR_XN_Pos)

#define MPU_AP_NO_ACCESS                        (0x00 << MPU_RASR_AP_Pos)
#define MPU_AP_PRIVILEGED_READ_WRITE            (0x01 << MPU_RASR_AP_Pos)
#define MPU_AP_UNPRIVILEGED_READONLY            (0x02 << MPU_RASR_AP_Pos)
#define MPU_AP_FULL_ACCESS                      (0x03 << MPU_RASR_AP_Pos)
#define MPU_AP_RES                              (0x04 << MPU_RASR_AP_Pos)
#define MPU_AP_PRIVILEGED_READONLY              (0x05 << MPU_RASR_AP_Pos)
#define MPU_AP_READONLY                         (0x06 << MPU_RASR_AP_Pos)
#define MPU_AP_READONLY2                        (0x07 << MPU_RASR_AP_Pos)

#define MPU_TEX_B000                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B001                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B010                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B011                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B100                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B101                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B110                            (0x01 << MPU_RASR_TEX_Pos)
#define MPU_TEX_B111                            (0x01 << MPU_RASR_TEX_Pos)

/*************************************************
 *      Memory type and its attribute
 *************************************************/
#define SHAREABLE       1
#define NON_SHAREABLE   0
/*********************************************************************************************************************************************************************
*   Memory Type Definition                          Memory TEX attribute            C attribute                     B attribute                     S attribute
**********************************************************************************************************************************************************************/

#define STRONGLY_ORDERED_SHAREABLE_TYPE      ((0x00 << MPU_RASR_TEX_Pos) | (DISABLE << MPU_RASR_C_Pos) | (DISABLE << MPU_RASR_B_Pos))     // DO not care //
#define SHAREABLE_DEVICE_TYPE                ((0x00 << MPU_RASR_TEX_Pos) | (DISABLE << MPU_RASR_C_Pos) | (ENABLE  << MPU_RASR_B_Pos))     // DO not care //
#define INNER_OUTER_NORMAL_WT_NWA_TYPE(x)    ((0x00 << MPU_RASR_TEX_Pos) | (ENABLE  << MPU_RASR_C_Pos) | (DISABLE << MPU_RASR_B_Pos) | (x << MPU_RASR_S_Pos))
#define INNER_OUTER_NORMAL_WB_NWA_TYPE(x)    ((0x00 << MPU_RASR_TEX_Pos) | (ENABLE  << MPU_RASR_C_Pos) | (ENABLE  << MPU_RASR_B_Pos) | (x << MPU_RASR_S_Pos))
#define INNER_OUTER_NORMAL_NOCACHE_TYPE(x)   ((0x01 << MPU_RASR_TEX_Pos) | (DISABLE << MPU_RASR_C_Pos) | (DISABLE << MPU_RASR_B_Pos) | (x << MPU_RASR_S_Pos))
#define INNER_OUTER_NORMAL_WB_RWA_TYPE(x)    ((0x01 << MPU_RASR_TEX_Pos) | (ENABLE  << MPU_RASR_C_Pos) | (ENABLE  << MPU_RASR_B_Pos) | (x << MPU_RASR_S_Pos))
#define NON_SHAREABLE_DEVICE_TYPE            ((0x02 << MPU_RASR_TEX_Pos) | (DISABLE << MPU_RASR_C_Pos) | (DISABLE << MPU_RASR_B_Pos))     // DO not care //

/*  Normal memory attributes with outer capability rules to Non_Cacable */

#define INNER_NORMAL_NOCACHE_TYPE(x)  ((0x04 << MPU_RASR_TEX_Pos) | (DISABLE  << MPU_RASR_C_Pos) | (DISABLE  << MPU_RASR_B_Pos) | (x << MPU_RASR_S_Pos))
#define INNER_NORMAL_WB_RWA_TYPE(x)   ((0x04 << MPU_RASR_TEX_Pos) | (DISABLE  << MPU_RASR_C_Pos) | (ENABLE  << MPU_RASR_B_Pos)  | (x << MPU_RASR_S_Pos))
#define INNER_NORMAL_WT_NWA_TYPE(x)   ((0x04 << MPU_RASR_TEX_Pos) | (ENABLE  << MPU_RASR_C_Pos)  | (DISABLE  << MPU_RASR_B_Pos) | (x << MPU_RASR_S_Pos))
#define INNER_NORMAL_WB_NWA_TYPE(x)   ((0x04 << MPU_RASR_TEX_Pos) | (ENABLE  << MPU_RASR_C_Pos)  | (ENABLE  << MPU_RASR_B_Pos)  | (x << MPU_RASR_S_Pos))


/* Default memory map
   Address range          Memory region          Memory type      Shareability   Cache policy
   0x00000000- 0x1FFFFFFF Code                   Normal           Non-shareable  WT
   0x20000000- 0x3FFFFFFF SRAM                   Normal           Non-shareable  WBWA
   0x40000000- 0x5FFFFFFF Peripheral             Device           Non-shareable  -
   0x60000000- 0x7FFFFFFF RAM                    Normal           Non-shareable  WBWA
   0x80000000- 0x9FFFFFFF RAM                    Normal           Non-shareable  WT
   0xA0000000- 0xBFFFFFFF Device                 Device           Shareable
   0xC0000000- 0xDFFFFFFF Device                 Device           Non Shareable
   0xE0000000- 0xFFFFFFFF System                  -                     -
   */

/**************** ITCM  *******************************/
#define ITCM_START_ADDRESS                  0x00000000UL
#define ITCM_END_ADDRESS                    0x003FFFFFUL

/********* IFLASH memory macros *********************/
#define IFLASH_START_ADDRESS                0x00400000UL
#define IFLASH_END_ADDRESS                  0x005FFFFFUL
#define IFLASH_PRIVILEGE_START_ADDRESS      (IFLASH_START_ADDRESS)
#define IFLASH_PRIVILEGE_END_ADDRESS        (IFLASH_START_ADDRESS + 0xFFF)
#define IFLASH_UNPRIVILEGE_START_ADDRESS    (IFLASH_PRIVILEGE_END_ADDRESS + 1)
#define IFLASH_UNPRIVILEGE_END_ADDRESS      (IFLASH_END_ADDRESS)

/**************** DTCM  *******************************/
#define DTCM_START_ADDRESS                  0x20000000UL
#define DTCM_END_ADDRESS                    0x203FFFFFUL

/******* SRAM memory macros ***************************/
#define SRAM_START_ADDRESS                  0x20400000UL
#define SRAM_END_ADDRESS                    0x2045FFFFUL

/************** Peripherals memory region macros ********/
#define PERIPHERALS_START_ADDRESS            0x40000000UL
#define PERIPHERALS_END_ADDRESS              0x5FFFFFFFUL

/******* Ext EBI memory macros ***************************/
#define EXT_EBI_START_ADDRESS                0x60000000UL
#define EXT_EBI_END_ADDRESS                  0x6FFFFFFFUL

/******* Ext-SRAM memory macros ***************************/
#define SDRAM_START_ADDRESS                  0x70000000UL
#define SDRAM_END_ADDRESS                    0x7FFFFFFFUL

/******* QSPI macros ***************************/
#define QSPI_START_ADDRESS                   0x80000000UL
#define QSPI_END_ADDRESS                     0x9FFFFFFFUL

/************** USBHS_RAM region macros ******************/
#define USBHSRAM_START_ADDRESS               0xA0100000UL
#define USBHSRAM_END_ADDRESS                 0xA01FFFFFUL

/*----------------------------------------------------------------------------
 *        Export functions
 *----------------------------------------------------------------------------*/
void MPU_Enable(uint32_t dwMPUEnable);
void MPU_SetRegion(uint32_t dwRegionBaseAddr, uint32_t dwRegionAttr);
void MPU_SetRegionNum(uint32_t dwRegionNum);
void MPU_DisableRegion(void);
uint32_t MPU_CalMPURegionSize(uint32_t dwActualSizeInBytes);
void MPU_UpdateRegions(uint32_t dwRegionNum, uint32_t dwRegionBaseAddr,	uint32_t dwRegionAttr);

#endif /* #ifndef _MMU_ */
