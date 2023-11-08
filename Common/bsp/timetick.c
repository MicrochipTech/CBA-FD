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

/**
 *  \file
 *  Implement the System Timer.
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include <sam.h>
#include <assert.h>
#include "board.h"
#if defined(FIRMWARE_CBA)
#include <FreeRTOS.h>
#include <task.h>
#endif
/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/
#define MAX_TIMER   4
/** Tick Counter united by ms */
static volatile uint32_t _dwTickCount = 0;
static uint8_t SysTickConfigured = 0;

/*----------------------------------------------------------------------------
 *         Exported Functions
 *----------------------------------------------------------------------------*/


/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  Increments the time-stamp counter.
 */
COMPILER_SECTION(".code_TCM") void TimeTick_Tick(void)
{
  _dwTickCount++;
}

/**
 *  \brief Configures the System Timer.
 *  Systick interrupt handler will generates 1ms interrupt and increase a
 *  tickCount.
 *  \note IRQ handler must be configured before invoking this function.
 */
uint32_t TimeTick_Configure(void)
{
	uint8_t Mdiv_Val;
	uint32_t Pck;
	_dwTickCount = 0;

//	TRACE_INFO("Configure system tick to get 1ms tick period.\r\n");
	/* check if there is MDIV value */
	Mdiv_Val = ((PMC->PMC_MCKR & PMC_MCKR_MDIV_Msk) >> PMC_MCKR_MDIV_Pos);

	if (Mdiv_Val == 0)
		Pck = SystemCoreClock/2;
	else if (Mdiv_Val == 3)
		Pck = SystemCoreClock/2 * Mdiv_Val;
	else
		Pck = SystemCoreClock/2 * (Mdiv_Val * 2);

	/* Configure SysTick for 1 ms. */
	if (SysTick_Config(Pck / 1000)) {
//		TRACE_ERROR("SysTick configuration error\r\n");
		SysTickConfigured = 0;
		return 1;
	}

	SysTickConfigured = 1;
	return 0;
}

void TimeTick_Disable(void)
{
  SysTick->CTRL = 0;
}

/**
 * \brief Get Delayed number of tick
 * \param startTick Start tick point.
 * \param endTick   End tick point.
 */
uint32_t GetDelayInTicks(uint32_t startTick, uint32_t endTick)
{
	if (endTick >= startTick) return (endTick - startTick);
	return (endTick + (0xFFFFFFFF - startTick) + 1);
}

/**
 * \brief Get Delayed number of tick
 * \param startTick Start tick point.
 * \param endTick   End tick point.
 */
uint32_t GetTicks(void)
{
#if defined(FIRMWARE_BOOTLOADER) || defined(FIRMWARE_CBA_DUT)
    assert(SysTickConfigured);
	return _dwTickCount;
#else
  return xTaskGetTickCount();
#endif
}

/**
 *  \brief Sync Wait for several ms
 *  \param dwMs    Waiting time in ms.
 */
void Wait(volatile uint32_t dwMs)
{
#if defined(FIRMWARE_BOOTLOADER) || defined(FIRMWARE_CBA_DUT)
	uint32_t dwStart;
    volatile uint32_t dwEnd;

	assert(SysTickConfigured);

	dwStart = _dwTickCount;
	dwEnd = _dwTickCount;

	while (GetDelayInTicks(dwStart, dwEnd) < dwMs)
		dwEnd = GetTicks();
#else
  vTaskDelay(dwMs);
#endif
}

/**
 *  \brief Sync Sleep for several ms
 *  \param dwMs    Sleeping time in ms.
 */
void Sleep(volatile uint32_t dwMs)
{
	uint32_t dwStart , dwEnd;

	assert(SysTickConfigured);

	__ASM("CPSIE   I");
	dwStart = _dwTickCount;
	dwEnd = _dwTickCount;

	do {
		if (GetDelayInTicks(dwStart, dwEnd) < dwMs)
			break;

		dwEnd = _dwTickCount;
		__ASM("WFI");
	} while (1);
}
