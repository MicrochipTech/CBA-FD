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

/** \file */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include "Media.h"

/*---------------------------------------------------------------------------
 *      Exported Global Variables
 *---------------------------------------------------------------------------*/

/** Number of medias which are effectively used. */
uint32_t gNbMedias = 0;

/*---------------------------------------------------------------------------
 *      Exported Functions
 *---------------------------------------------------------------------------*/


/**
 *  \brief  Locks all the regions in the given address range.
 *  \param  media    Pointer to a Media instance
 *  \param  start  Start address of lock range.
 *  \param  end  End address of lock range.
 *  \param  pActualStart  Start address of the actual lock range (optional).
 *  \param  pActualEnd  End address of the actual lock range (optional).
 *  \return 0 if successful; otherwise returns an error code.
 */
extern uint8_t MED_Lock(sMedia *pMedia,
						 uint32_t start, uint32_t end,
						 uint32_t *pActualStart, uint32_t *pActualEnd)
{
	if (pMedia->lock)
		return pMedia->lock(pMedia, start, end, pActualStart, pActualEnd);
	else
		return MED_STATUS_SUCCESS;
}

/**
 *  \brief  Unlocks all the regions in the given address range
 *  \param  media    Pointer to a Media instance
 *  \param start  Start address of unlock range.
 *  \param end  End address of unlock range.
 *  \param pActualStart  Start address of the actual unlock range (optional).
 *  \param pActualEnd  End address of the actual unlock range (optional).
 *  \return 0 if successful; otherwise returns an error code.
 */
extern uint8_t MED_Unlock(sMedia *pMedia,
						   uint32_t start, uint32_t end,
						   uint32_t *pActualStart, uint32_t *pActualEnd)
{
	if (pMedia->unlock)
		return pMedia->unlock(pMedia, start, end, pActualStart, pActualEnd);
	else
		return MED_STATUS_SUCCESS;
}

/**
 *  \brief
 *  \param  media Pointer to the Media instance to use
 */
extern uint8_t MED_Flush(sMedia *pMedia)
{
	if (pMedia->flush)
		return pMedia->flush(pMedia);
	else
		return MED_STATUS_SUCCESS;
}

/**
 *  \brief  Invokes the interrupt handler of the specified media
 *  \param  media Pointer to the Media instance to use
 */
extern void MED_Handler(sMedia *pMedia)
{
	if (pMedia->handler)
		pMedia->handler(pMedia);
}

/**
 *  \brief  Reset the media interface to un-configured state.
 *  \param  media Pointer to the Media instance to use
 */
extern void MED_DeInit(sMedia *pMedia)
{
	pMedia->state = MED_STATE_NOT_READY;
}

/**
 *  \brief  Check if the Media instance is ready to use.
 *  \param  media Pointer to the Media instance to use
 */
extern uint8_t MED_IsInitialized(sMedia *pMedia)
{
	return (pMedia->state != MED_STATE_NOT_READY);
}

/**
 *  \brief  Check if the Media instance is busy in transfer.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint8_t MED_IsBusy(sMedia *pMedia)
{
	return (pMedia->state == MED_STATE_BUSY);
}

/**
 *  \brief  Check if the Media supports mapped reading.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint8_t MED_IsMappedRDSupported(sMedia *pMedia)
{
	return pMedia->mappedRD;
}

/**
 *  \brief  Check if the Media supports mapped writing.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint8_t MED_IsMappedWRSupported(sMedia *pMedia)
{
	return pMedia->mappedRD;
}

/**
 *  \brief  Check if the Media is write protected.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint8_t MED_IsProtected(sMedia *pMedia)
{
	return pMedia->protected;
}

/**
 *  \brief  Return current state of the Media.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint8_t MED_GetState(sMedia *pMedia)
{
	return pMedia->state;
}

/**
 *  \brief  Return block size in bytes.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint32_t MED_GetBlockSize(sMedia *pMedia)
{
	return pMedia->blockSize;
}

/**
 *  \brief  Return Media size in number of blocks.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint32_t MED_GetSize(sMedia *pMedia)
{
	return pMedia->size;
}

/**
 *  \brief  Return mapped memory address for a block on media.
 *  \param  pMedia Pointer to the Media instance to use
 */
extern uint32_t MED_GetMappedAddress(sMedia *pMedia,
									 uint32_t dwBlk)
{
	return ((pMedia->baseAddress + dwBlk) * pMedia->blockSize);
}

/**
 *  \brief  Handle interrupts on specified media
 *  \param  pMedia    List of media
 *  \param  bNumMedia Number of media in list
 *  \see    S_media
 */
extern void MED_HandleAll(sMedia *pMedia, uint8_t bNumMedia)
{
	uint32_t i;

	for (i = 0; i < bNumMedia; i++)
		MED_Handler(&(pMedia[i]));
}
