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
 * \file
 *
 * Implementation of media layer for the SdCard.
 *
 */

/*------------------------------------------------------------------------------
 *         Headers
 *------------------------------------------------------------------------------*/
#include "board.h"
#include "Media.h"
#include "MEDSdcard.h"
#include "libsdmmc.h"
#include <assert.h>
#include <string.h>
#include <sdmmc_trace.h>
/*------------------------------------------------------------------------------
 *         Constants
 *------------------------------------------------------------------------------*/

/** Number of SD Slots */
#define NUM_SD_SLOTS            1
/** Default block size for SD/MMC card access */
#define SD_BLOCK_SIZE       512
/**
 * \brief  Reads a specified amount of data from a SDCARD memory
 * \param  media    Pointer to a Media instance
 * \param  address  Address of the data to read
 * \param  data     Pointer to the buffer in which to store the retrieved
 *                   data
 * \param  length   Length of the buffer
 * \param  callback Optional pointer to a callback function to invoke when
 *                   the operation is finished
 * \param  argument Optional pointer to an argument for the callback
 * \return Operation result code
 */
uint8_t MEDSdcard_Read(
    sMedia *media,
    uint32_t address,
    void* data,
    uint32_t length,
    MediaCallback callback,
    void* argument)
{
	uint8_t error;

	// Check that the media is ready
	if (media->state != MED_STATE_READY)
  {
		TRACE_INFO("Media busy\r\n");
		return MED_STATUS_BUSY;
	}

	// Check that the data to read is not too big
	if ((length + address) > media->size)
  {
		TRACE_WARNING("MEDSdcard_Read: Data too big: %d, %d\r\n", (int)length, (int)address);
		return MED_STATUS_ERROR;
	}

	// Enter Busy state
	media->state = MED_STATE_BUSY;

	error = SD_Read((sSdCard *)media->interface, address, data, length, NULL, NULL);

	// Leave the Busy state
	media->state = MED_STATE_READY;

	// Invoke callback
	if (callback != 0)
  {
    callback(argument, error, 0, 0);
  }

	return error;
}

/**
 * \brief  Writes data on a SDRAM media
 * \param  media    Pointer to a Media instance
 * \param  address  Address at which to write
 * \param  data     Pointer to the data to write
 * \param  length   Size of the data buffer
 * \param  callback Optional pointer to a callback function to invoke when
 *                   the write operation terminates
 * \param  argument Optional argument for the callback function
 * \return Operation result code
 * \see    Media
 * \see    MediaCallback
 */
uint8_t MEDSdcard_Write(
    sMedia* media,
    uint32_t address,
    void* data,
    uint32_t length,
    MediaCallback callback,
    void* argument)
{
	uint8_t error;

	// Check that the media if ready
	if (media->state != MED_STATE_READY)
  {
		TRACE_WARNING("MEDSdcard_Write: Media is busy\r\n");
		return MED_STATUS_BUSY;
	}

	// Check that the data to write is not too big
	if ((length + address) > media->size)
  {
		TRACE_WARNING("MEDSdcard_Write: Data too big\r\n");
		return MED_STATUS_ERROR;
	}

	// Put the media in Busy state
	media->state = MED_STATE_BUSY;

	error = SD_Write((sSdCard *)media->interface, address, data, length, NULL, NULL);

	// Leave the Busy state
	media->state = MED_STATE_READY;

	// Invoke the callback if it exists
	if (callback != 0)
  {
		callback(argument, error, 0, 0);
  }
    
	return error;
}

/**
 * \brief  Initializes a Media instance
 * \param  media Pointer to the Media instance to initialize
 * \return 1 if success.
 */
uint8_t MEDSdcard_Initialize(sMedia *media, sSdCard *pSdDrv)
{
	TRACE_INFO("MEDSdcard init\r\n");
  
	// Initialize media fields
	media->interface = pSdDrv;
	media->write = 0;
	media->read = 0;
	media->lock = 0;
	media->unlock = 0;
	media->handler = 0;
	media->flush = 0;

	media->blockSize = SD_BLOCK_SIZE;
	media->baseAddress = 0;
	media->size = pSdDrv->dwNbBlocks;

	media->mappedRD  = 0;
	media->mappedWR  = 0;
	media->removable = 1;

	media->state = MED_STATE_READY;

	media->transfer.data = 0;
	media->transfer.address = 0;
	media->transfer.length = 0;
	media->transfer.callback = 0;
	media->transfer.argument = 0;

	return 1;
}

