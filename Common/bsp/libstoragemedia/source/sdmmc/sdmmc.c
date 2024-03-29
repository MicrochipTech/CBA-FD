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

/** \addtogroup sdmmc_api
 *  @{
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include <FreeRTOS.h>
#include <semphr.h>
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include "libsdmmc.h"
#include "sdmmc_trace.h"
#include "mcid.h"

/*----------------------------------------------------------------------------
 *         Global variables
 *----------------------------------------------------------------------------*/
 //SemaphoreHandle_t sdMutex = NULL;
 //StaticSemaphore_t sdMutexBuffer;

static SemaphoreHandle_t sdSemaphore = NULL;
static StaticSemaphore_t sdSemaphoreBuffer;

static uint8_t _StopCmd(sSdCard *pSd);
static uint8_t Cmd12(sSdCard *pSd, uint32_t *pStatus);
static uint8_t Cmd55(sSdCard *pSd, uint16_t cardAddr);
#define STOPINCB 1

/*----------------------------------------------------------------------------
 *         Local constants
 *----------------------------------------------------------------------------*/

/** \addtogroup sdmmc_status_bm SD/MMC Status register constants
 *      @{*/
#define STATUS_APP_CMD          (1UL << 5)
#define STATUS_SWITCH_ERROR     (1UL << 7)
#define STATUS_READY_FOR_DATA   (1UL << 8)
#define STATUS_IDLE             (0UL << 9)
#define STATUS_READY            (1UL << 9)
#define STATUS_IDENT            (2UL << 9)
#define STATUS_STBY             (3UL << 9)
#define STATUS_TRAN             (4UL << 9)
#define STATUS_DATA             (5UL << 9)
#define STATUS_RCV              (6UL << 9)
#define STATUS_PRG              (7UL << 9)
#define STATUS_DIS              (8UL << 9)
#define STATUS_STATE            (0xFUL << 9)
#define STATUS_ERASE_RESET       (1UL << 13)
#define STATUS_WP_ERASE_SKIP     (1UL << 15)
#define STATUS_CIDCSD_OVERWRITE  (1UL << 16)
#define STATUS_OVERRUN           (1UL << 17)
#define STATUS_UNERRUN           (1UL << 18)
#define STATUS_ERROR             (1UL << 19)
#define STATUS_CC_ERROR          (1UL << 20)
#define STATUS_CARD_ECC_FAILED   (1UL << 21)
#define STATUS_ILLEGAL_COMMAND   (1UL << 22)
#define STATUS_COM_CRC_ERROR     (1UL << 23)
#define STATUS_UN_LOCK_FAILED    (1UL << 24)
#define STATUS_CARD_IS_LOCKED    (1UL << 25)
#define STATUS_WP_VIOLATION      (1UL << 26)
#define STATUS_ERASE_PARAM       (1UL << 27)
#define STATUS_ERASE_SEQ_ERROR   (1UL << 28)
#define STATUS_BLOCK_LEN_ERROR   (1UL << 29)
#define STATUS_ADDRESS_MISALIGN  (1UL << 30)
#define STATUS_ADDR_OUT_OR_RANGE (1UL << 31)

#define STATUS_STOP ((uint32_t)(STATUS_CARD_IS_LOCKED \
								 | STATUS_COM_CRC_ERROR \
								 | STATUS_ILLEGAL_COMMAND \
								 | STATUS_CC_ERROR \
								 | STATUS_ERROR \
								 | STATUS_STATE \
								 | STATUS_READY_FOR_DATA))

#define STATUS_WRITE ((uint32_t)(STATUS_ADDR_OUT_OR_RANGE \
								  | STATUS_ADDRESS_MISALIGN \
								  | STATUS_BLOCK_LEN_ERROR \
								  | STATUS_WP_VIOLATION \
								  | STATUS_CARD_IS_LOCKED \
								  | STATUS_COM_CRC_ERROR \
								  | STATUS_ILLEGAL_COMMAND \
								  | STATUS_CC_ERROR \
								  | STATUS_ERROR \
								  | STATUS_ERASE_RESET \
								  | STATUS_STATE \
								  | STATUS_READY_FOR_DATA))

#define STATUS_READ  ((uint32_t)(STATUS_ADDR_OUT_OR_RANGE \
								  | STATUS_ADDRESS_MISALIGN \
								  | STATUS_BLOCK_LEN_ERROR \
								  | STATUS_CARD_IS_LOCKED \
								  | STATUS_COM_CRC_ERROR \
								  | STATUS_ILLEGAL_COMMAND \
								  | STATUS_CARD_ECC_FAILED \
								  | STATUS_CC_ERROR \
								  | STATUS_ERROR \
								  | STATUS_ERASE_RESET \
								  | STATUS_STATE \
								  | STATUS_READY_FOR_DATA))

#define STATUS_SD_SWITCH ((uint32_t)(STATUS_ADDR_OUT_OR_RANGE \
									  | STATUS_CARD_IS_LOCKED \
									  | STATUS_COM_CRC_ERROR \
									  | STATUS_ILLEGAL_COMMAND \
									  | STATUS_CARD_ECC_FAILED \
									  | STATUS_CC_ERROR \
									  | STATUS_ERROR \
									  | STATUS_UNERRUN \
									  | STATUS_OVERRUN \
									  /*| STATUS_STATE*/))

#define STATUS_MMC_SWITCH ((uint32_t)(STATUS_CARD_IS_LOCKED \
									   | STATUS_COM_CRC_ERROR \
									   | STATUS_ILLEGAL_COMMAND \
									   | STATUS_CC_ERROR \
									   | STATUS_ERROR \
									   | STATUS_ERASE_RESET \
									   /*| STATUS_STATE*/ \
									   /*| STATUS_READY_FOR_DATA*/ \
									   | STATUS_SWITCH_ERROR))
/**     @}*/

/** \addtogroup sdio_status_bm SDIO Status definitions
 *      @{*/
/** The CRC check of the previous command failed. */
#define SDIO_COM_CRC_ERROR   (1UL << 15)
/** Command not legal for the card state. */
#define SDIO_ILLEGAL_COMMAND (1UL << 14)

/** Status bits mask for SDIO R6 */
#define STATUS_SDIO_R6  (SDIO_COM_CRC_ERROR \
						 | SDIO_ILLEGAL_COMMAND \
						 | SDIO_R6_ERROR)
/** Status bits mask for SDIO R5 */
#define STATUS_SDIO_R5  (0/*SDIO_R5_STATE*/ \
						 | SDIO_R5_ERROR \
						 | SDIO_R5_FUNCN_ERROR \
						 | SDIO_R5_OUT_OF_RANGE)
/**     @}*/

/*----------------------------------------------------------------------------
 *         Macros
 *----------------------------------------------------------------------------*/

/** Return SD/MMC card address */
#define CARD_ADDR(pSd)          (pSd->wAddress)

/** Return SD/MMC card block size (Default size now, 512B) */
#define BLOCK_SIZE(pSd)         (pSd->wCurrBlockLen)

/** Convert block address to SD/MMC command parameter */
#define SD_ADDRESS(pSd, address) \
	(((pSd)->dwTotalSize >= 0xFFFFFFFF) ? \
	  (address):((address) << 9))

/** Check if SD Spec version 1.10 or later */
// #define SD_IsVer1_10(pSd) (SD_SCR_SD_SPEC(pSd->SCR) >= SD_SCR_SD_SPEC_1_10)

/** Check if SD card support HS mode (1.10 or later) */
#define SD_IsHsModeSupported(pSd)  \
	(/*SD_IsVer1_10(pSd)||*/(SD_CSD_STRUCTURE(pSd->CSD)>=1))

/** Check if SD card support 4-bit mode (All SD card) */
#define SD_IsBusModeSupported(pSd) (1)

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

COMPILER_SECTION("data_TCM") COMPILER_ALIGNED(32) static uint32_t switchStatus[512 / 32];

/** SD/MMC transfer rate unit codes (10K) list */
static const uint32_t sdmmcTransUnits[7] = {
	10, 100, 1000, 10000,
	0, 0, 0
};

/** SD transfer multiplier factor codes (1/10) list */
static const uint32_t sdTransMultipliers[16] = {
	0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80
};

/** Flag for dumping SSR register */
static uint8_t ssr_is_dump = 0;

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 */
static void _ResetCmd(sSdmmcCommand *pCmd)
{
	memset(pCmd, 0, sizeof(sSdmmcCommand));
}

void giveSdSemaphore(uint32_t status, void *pArg)
{
  xSemaphoreGiveFromISR(sdSemaphore, 0);
}

/**
 */
static uint8_t _SendCmd(sSdCard *pSd, fSdmmcCallback fCallback, void *pCbArg)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t bRc;

	pCmd->fCallback = giveSdSemaphore;
	pCmd->pArg      = pCbArg;
	bRc = MCID_SendCmd(pSd->pDrv, pCmd);      // Send request to sdcard (dma processing etc.)

  if(bRc == SDMMC_OK)
  {
    xSemaphoreTake(sdSemaphore, portMAX_DELAY);
  
#ifdef STOPINCB
    if(!pSd->bCmd23)
    {
      if ((pSd->bState == SDMMC_STATE_DATA_RD) || (pSd->bState == SDMMC_STATE_DATA_WR))
      {
        pSd->bState = SDMMC_STATE_TRAN;
        _StopCmd(pSd);
      }
    }  
#endif

	  return pCmd->bStatus;
  }
  
  return bRc;
}

/**
 */
static uint8_t _HwSetBusWidth(sSdCard *pSd, uint8_t newWidth)
{
	void *pDrv = pSd->pDrv;
	uint32_t busWidth = newWidth;
	uint32_t rc;

	rc = MCID_IOCtrl(pDrv, SDMMC_IOCTL_SET_BUSMODE, (uint32_t)&busWidth);
	return rc;
}

/**
 */
static uint8_t _HwGetHsMode(sSdCard *pSd)
{
	void *pDrv = pSd->pDrv;
	uint32_t hsMode = 0;
	MCID_IOCtrl(pDrv, SDMMC_IOCTL_GET_HSMODE, (uint32_t)&hsMode);
	return hsMode;
}

/**
 */
static uint8_t _HwSetHsMode(sSdCard *pSd, uint8_t newHsMode)
{
	void *pDrv = pSd->pDrv;
	uint32_t hsMode = newHsMode;
	uint32_t rc = MCID_IOCtrl(pDrv, SDMMC_IOCTL_SET_HSMODE, (uint32_t)&hsMode);
	return rc;
}

/**
 */
static uint8_t _HwSetClock(sSdCard *pSd, uint32_t *pIoValClk)
{
	void *pDrv = pSd->pDrv;
	uint32_t clock = *pIoValClk;
	uint32_t rc;
	rc = MCID_IOCtrl(pDrv, SDMMC_IOCTL_SET_CLOCK, (uint32_t)&clock);

	if (rc == SDMMC_SUCCESS || rc == SDMMC_CHANGED)
		*pIoValClk = clock;

	return rc;
}

/**
 */
static uint8_t _HwReset(sSdCard *pSd)
{
	void *pDrv = pSd->pDrv;
	uint32_t rc;
	rc = MCID_IOCtrl(pDrv, SDMMC_IOCTL_RESET, 0);
	return rc;
}

/**
 * Find SDIO ManfID, Fun0 tuple.
 * \param pSd         Pointer to \ref sSdCard instance.
 * \param address     Search area start address.
 * \param size        Search area size.
 * \param pAddrManfID Pointer to ManfID address value buffer.
 * \param pAddrFunc0  Pointer to Func0 address value buffer.
 */
static uint8_t SdioFindTuples(sSdCard *pSd,
							  uint32_t address, uint32_t size,
							  uint32_t *pAddrManfID,
							  uint32_t *pAddrFunc0)
{
	uint8_t error, tmp[3];
	uint32_t addr = address;
	uint8_t  flagFound = 0; /* 1:Manf, 2:Func0 */
	uint32_t addManfID = 0, addFunc0 = 0;

	for (; flagFound != 3;) {
		error = SDIO_ReadDirect(pSd, SDIO_CIA, addr, tmp, 3);

		if (error) {
			TRACE_ERROR("SdioFindTuples.RdDirect: %d\r\n", error);
			return error;
		}

		/* End */
		if (tmp[0] == CISTPL_END) break;
		/* ManfID */
		else if (tmp[0] == CISTPL_MANFID) {
			flagFound |= 1; addManfID = addr;
		}
		/* Func0 */
		else if (tmp[0] == CISTPL_FUNCE && tmp[2] == 0x00) {
			flagFound |= 2; addFunc0 = addr;
		}

		/* Tuple error ? */
		if (tmp[1] == 0) break;

		/* Next address */
		addr += (tmp[1] + 2);

		if (addr > (address + size)) break;
	}

	if (pAddrManfID) *pAddrManfID = addManfID;

	if (pAddrFunc0)  *pAddrFunc0  = addFunc0;

	return 0;
}

/**
 * \brief Decode Trans Speed Value
 * \param code The trans speed code value.
 * \param unitCodes  Unit list in 10K, 0 as unused value.
 * \param multiCodes Multiplier list in 1/10, index 1 ~ 15 is valid.
 */
static uint32_t SdmmcDecodeTransSpeed(uint32_t code,
									  const uint32_t *unitCodes,
									  const uint32_t *multiCodes)
{
	uint32_t speed;
	uint8_t unitI, mulI;

	/* Unit code is valid ? */
	unitI = (code & 0x7);

	if (unitCodes[unitI] == 0) return 0;

	/* Multi code is valid ? */
	mulI = (code >> 3) & 0xF;

	if (multiCodes[mulI] == 0) return 0;

	speed = unitCodes[unitI] * multiCodes[mulI];
	return speed;
}

/**
 * Initialization delay: The maximum of 1 msec, 74 clock cycles and supply ramp
 * up time.
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd Pointer to \ref sSdCard instance.
 */
static inline uint8_t Pon(sSdCard *pSd)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t bRc;

	TRACE_DEBUG("PwrOn()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_POWERONINIT;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Resets all cards to idle state
 * \param pSd Pointer to \ref sSdCard instance.
 * \param arg  Argument used.
 * \return the command transfer result (see SendMciCommand).
 */
static inline uint8_t Cmd0(sSdCard *pSd, uint8_t arg)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t bRc;

	TRACE_DEBUG("Cmd0()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(0);
	pCmd->dwArg      = arg;
	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * MMC send operation condition command.
 * Sends host capacity support information and activates the card's
 * initialization process.
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd Pointer to \ref sSdCard instance.
 * \param pOCR Pointer to fill OCR value to send and get.
 */
static inline uint8_t Cmd1(sSdCard *pSd, uint8_t *pHd)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;
	uint32_t dwArg;

	TRACE_DEBUG("Cmd1()\r\n");
	_ResetCmd(pCmd);

	dwArg = SD_HOST_VOLTAGE_RANGE | MMC_OCR_ACCESS_SECTOR;

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(3)
					   | SDMMC_CMD_bmOD;
	pCmd->bCmd       = 1;
	pCmd->dwArg      = dwArg;
	pCmd->pResp      = &dwArg;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);

	/* Check result */
	if (bRc) return bRc;

	if (dwArg & SD_OCR_BUSY) {
		*pHd = 0;

		if ((dwArg & MMC_OCR_ACCESS_MODE) == MMC_OCR_ACCESS_SECTOR)
			*pHd = 1;

		return 0;
	}

	return SDMMC_ERROR_NOT_INITIALIZED;
}

/**
 * Asks any card to send the CID numbers
 * on the CMD line (any card that is
 * connected to the host will respond)
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd  Pointer to a SD card driver instance.
 * \param pCID Pointer to buffer for storing the CID numbers.
 */
static inline uint8_t Cmd2(sSdCard *pSd)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd2()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(2)
					   | SDMMC_CMD_bmOD;
	pCmd->bCmd       = 2;
	pCmd->pResp      = pSd->CID;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Ask the SD card to publish a new relative address (RCA)
 *         or
 * Assign relative address to the MMC card
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd   Pointer to a SD card driver instance.
 * \param pRsp  Pointer to buffer to fill response (address on 31:16).
 */
static uint8_t Cmd3(sSdCard *pSd)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint32_t dwResp;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd3()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(6) | SDMMC_CMD_bmOD;
	pCmd->bCmd       = 3;
	pCmd->pResp      = &dwResp;
  
  bRc = _SendCmd(pSd, NULL, NULL);
  if (bRc == SDMMC_OK)
  {
    CARD_ADDR(pSd) = dwResp >> 16;
  }  

	return bRc;
}

/**
 * SDIO SEND OPERATION CONDITION (OCR) command.
 * Sends host capacity support information and activates the card's
 * initialization process.
 * \return The command transfer result (see SendMciCommand).
 * \param pSd     Pointer to a SD/MMC card driver instance.
 * \param pIo     Pointer to data sent as well as response buffer (32bit).
 */
static uint8_t Cmd5(sSdCard *pSd, uint32_t *pIo)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd5()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(4) | SDMMC_CMD_bmOD;
	pCmd->bCmd       = 5;
	pCmd->dwArg      = *pIo;
	pCmd->pResp      = pIo;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Switches the mode of operation of the selected card.
 * CMD6 is valid under the "trans" state.
 * \return The command transfer result (see SendMciCommand).
 * \param  pSd         Pointer to a SD/MMC card driver instance.
 * \param  pSwitchArg  Pointer to a MmcCmd6Arg instance.
 * \param  pStatus     Pointer to where the 512bit status is returned.
 * \param  pResp       Pointer to where the response is returned.
 */
static uint8_t SdCmd6(sSdCard *pSd,
					  const void *pSwitchArg,
					  uint32_t    *pStatus,
					  uint32_t    *pResp)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;
	SdCmd6Arg *pSdSwitch;

	assert(pSd);
	assert(pSwitchArg);

	TRACE_DEBUG("Cmd6()\r\n");
	_ResetCmd(pCmd);

	pSdSwitch = (SdCmd6Arg *)pSwitchArg;
	pCmd->bCmd       = 6;
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATARX(1);
	pCmd->dwArg      = (pSdSwitch->mode << 31)
					   | (pSdSwitch->reserved << 30)
					   | (pSdSwitch->reserveFG6 << 20)
					   | (pSdSwitch->reserveFG5 << 16)
					   | (pSdSwitch->reserveFG4 << 12)
					   | (pSdSwitch->reserveFG3 <<  8)
					   | (pSdSwitch->command << 4)
					   | (pSdSwitch->accessMode << 0);

	if (pStatus) {
		pCmd->wBlockSize = 512 / 8;
		pCmd->wNbBlocks  = 1;
		pCmd->pData      = (uint8_t *)pStatus;
	}

	pCmd->pResp = pResp;

	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Switches the mode of operation of the selected card or
 * Modifies the EXT_CSD registers.
 * CMD6 is valid under the "trans" state.
 * \return The command transfer result (see SendMciCommand).
 * \param  pSd         Pointer to a SD/MMC card driver instance.
 * \param  pSwitchArg  Pointer to a MmcCmd6Arg instance.
 * \param  pResp       Pointer to where the response is returned.
 */
static uint8_t MmcCmd6(sSdCard *pSd,
					   const void *pSwitchArg,
					   uint32_t    *pResp)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;
	MmcCmd6Arg *pMmcSwitch;

	assert(pSd);
	assert(pSwitchArg);

	TRACE_DEBUG("Cmd6()\r\n");
	_ResetCmd(pCmd);

	pMmcSwitch = (MmcCmd6Arg *)pSwitchArg;
	pCmd->bCmd       = 6;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(1);
	pCmd->dwArg      =   (pMmcSwitch->access << 24)
						 | (pMmcSwitch->index  << 16)
						 | (pMmcSwitch->value  <<  8)
						 | (pMmcSwitch->cmdSet <<  0);
	pCmd->pResp      = pResp;

	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Command toggles a card between the
 * stand-by and transfer states or between
 * the programming and disconnect states.
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param cardAddr  Relative Card Address (0 deselects all).
 */
static uint8_t Cmd7(sSdCard *pSd, uint16_t address)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd7()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(address ? 1 : 0);
	pCmd->bCmd       = 7;
	pCmd->dwArg      = address << 16;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Sends SD Memory Card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 * Should be performed at initialization time to detect the card type.
 * \param pSd             Pointer to a SD card driver instance.
 * \param supplyVoltage   Expected supply voltage(SD).
 * \return 0 if successful;
 *         otherwise returns SD_ERROR_NORESPONSE if the card did not answer
 *         the command, or SDMMC_ERROR.
 */
static uint8_t SdCmd8(sSdCard *pSd,
					  uint8_t supplyVoltage)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint32_t dwResp;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd8()\r\n");
	_ResetCmd(pCmd);

	/* Fill command information */
	pCmd->bCmd       = 8;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(7) | SDMMC_CMD_bmOD;
	pCmd->dwArg      = (supplyVoltage << 8) | (0xAA);
	pCmd->pResp      = &dwResp;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);

	/* Check result */
	if (bRc == SDMMC_ERROR_NORESPONSE)
		return SDMMC_ERROR_NORESPONSE;
	/* SD_R7
	 * Bit 0 - 7: check pattern (echo-back)
	 * Bit 8 -11: voltage accepted
	 */
	else if (!bRc &&
			 ((dwResp & 0x00000FFF) == ((uint32_t)(supplyVoltage << 8) | 0xAA)))
		return 0;
	else
		return SDMMC_ERROR;
}

/**
 * Addressed card sends its card-specific
 * data (CSD) on the CMD line.
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param cardAddr  Card Relative Address.
 * \param pCSD      Pointer to buffer for CSD data.
 */
static uint8_t Cmd9(sSdCard *pSd)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd9()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(2);
	pCmd->bCmd       = 9;
	pCmd->dwArg      = CARD_ADDR(pSd) << 16;
	pCmd->pResp      = pSd->CSD;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

static uint8_t Acmd51(sSdCard* pSd)
{
	Cmd55(pSd, CARD_ADDR(pSd));
  
  sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;
  uint32_t status[2];
  status[0] = 0;
  status[1] = 0;

	TRACE_DEBUG("Acmd51()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATARX(1);
	pCmd->bCmd       = 51;
	pCmd->dwArg      = 0;
	pCmd->pResp      = status;
  pCmd->pData      = (uint8_t*)pSd->SCR;
  pCmd->wBlockSize = 8;
  pCmd->wNbBlocks = 1;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);

  /* ACMD51 (Get SCR) requires the card to be in data state. The card will return to transfer state if the operation has completed successfully. */
  /* If there was an error during ACMD51 processing, send a CMD12 (Stop) which puts the card back into transfer state. */
  if(bRc != SDMMC_OK)
  {
    Cmd12(pSd, status);  
  }
  
  pSd->SCR[0] = __REV(pSd->SCR[0]);
  
	return bRc;
}

/*
	pCmd->bCmd       = 41;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(3);
	pCmd->dwArg      = *pOpIo;
	pCmd->pResp      = pOpIo;
*/

/**
 * Forces the card to stop transmission
 * \param pSd      Pointer to a SD card driver instance.
 * \param pStatus  Pointer to a status variable.
 */
static uint8_t Cmd12(sSdCard *pSd, uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd12()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->bCmd       = 12;
	pCmd->cmdOp.wVal = SDMMC_CMD_CSTOP
					   | SDMMC_CMD_bmBUSY;
	pCmd->pResp      = pStatus;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Addressed card sends its status register.
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param pStatus   Pointer to a status variable.
 */
static uint8_t Cmd13(sSdCard *pSd, uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd13()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->bCmd       = 13;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(1);
	pCmd->dwArg      = CARD_ADDR(pSd) << 16;
	pCmd->pResp      = pStatus;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * In the case of a Standard Capacity SD Memory Card, this command sets the
 * block length (in bytes) for all following block commands
 * (read, write, lock).
 * Default block length is fixed to 512 Bytes.
 * Set length is valid for memory access commands only if partial block read
 * operation are allowed in CSD.
 * In the case of a High Capacity SD Memory Card, block length set by CMD16
 * command does not affect the memory read and write commands. Always 512
 * Bytes fixed block length is used. This command is effective for LOCK_UNLOCK
 * command. In both cases, if block length is set larger than 512Bytes, the
 * card sets the BLOCK_LEN_ERROR bit.
 * \param pSd  Pointer to a SD card driver instance.
 * \param blockLength  Block length in bytes.
 */
static uint8_t Cmd16(sSdCard *pSd, uint16_t blkLen)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd16()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(1);
	pCmd->bCmd       = 16;
	pCmd->dwArg      = blkLen;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Read single block command
 * \param pSd  Pointer to a SD card driver instance.
 * \param pData     Pointer to the DW aligned buffer to be filled.
 * \param address   Data Address on SD/MMC card.
 * \param pStatus   Pointer response buffer as status return.
 * \param fCallback Pointer to optional callback invoked on command end.
 *                  NULL:    Function return until command finished.
 *                  Pointer: Return immediately and invoke callback at end.
 *                  Callback argument is fixed to a pointer to sSdCard instance.
 */
static uint8_t Cmd17(sSdCard *pSd,
					 uint8_t *pData,
					 uint32_t address,
					 uint32_t *pStatus,
					 fSdmmcCallback callback)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd17()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATARX(1);
	pCmd->bCmd       = 17;
	pCmd->dwArg      = address;
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = BLOCK_SIZE(pSd);
	pCmd->wNbBlocks  = 1;
	pCmd->pData      = pData;
	pCmd->fCallback  = callback;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * A host reads the reversed bus testing data pattern from a card
 * \param pSd  Pointer to a SD card driver instance.
 * \param pData     Pointer to the DW aligned buffer to be filled.
 * \param len       Length of data in byte
 * \param pStatus   Pointer response buffer as status return.
 */
static uint8_t Cmd14(sSdCard *pSd,
					 uint8_t *pData,
					 uint8_t len,
					 uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd14()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATARX(1);
	pCmd->bCmd       = 14;
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = len;
	pCmd->wNbBlocks  = 1;
	pCmd->pData      = pData;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * A host sends the bus test data pattern to a card.
 * \param pSd  Pointer to a SD card driver instance.
 * \param pData     Pointer to the DW aligned buffer to be filled.
 * \param len       Length of data in byte
 * \param pStatus   Pointer response buffer as status return.
*/
static uint8_t Cmd19(sSdCard *pSd,
					 uint8_t *pData,
					 uint8_t len,
					 uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd19()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATATX(1);
	pCmd->bCmd       = 19;
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = len;
	pCmd->wNbBlocks  = 1;
	pCmd->pData      = pData;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Continuously transfers datablocks from card to host until interrupted by a
 * STOP_TRANSMISSION command.
 * \param pSd       Pointer to a SD card driver instance.
 * \param nbBlocks  Number of blocks to send.
 * \param pData     Pointer to the DW aligned buffer to be filled.
 * \param address   Data Address on SD/MMC card.
 * \param pStatus   Pointer to the response status.
 * \param fCallback Pointer to optional callback invoked on command end.
 *                  NULL:    Function return until command finished.
 *                  Pointer: Return immediately and invoke callback at end.
 *                  Callback argument is fixed to a pointer to sSdCard instance.
 */
static uint8_t Cmd18(sSdCard *pSd,
					 uint16_t nbBlock,
					 uint8_t *pData,
					 uint32_t address,
					 uint32_t *pStatus,
					 fSdmmcCallback callback)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd18()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATARX(1);
	pCmd->bCmd       = 18;
	pCmd->dwArg      = address;
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = BLOCK_SIZE(pSd);
	pCmd->wNbBlocks  = nbBlock;
	pCmd->pData      = pData;
	pCmd->fCallback  = callback;
	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}


/**
 * Defines the number of blocks (read/write) and the reliable writer parameter
 * (write) for a block read or write command
 * data (CSD) on the CMD line.
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param write     Write Request parameter.
 * \param blocks    number of blocks.
 */
static uint8_t Cmd23(sSdCard *pSd , uint8_t write, uint32_t blocks, uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd23()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(1);
	pCmd->bCmd       = 23;
	pCmd->wNbBlocks  = 0;
	pCmd->dwArg      = write << 31 | blocks;
	pCmd->pResp      = pStatus;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Write single block command
 * \param pSd  Pointer to a SD card driver instance.
 * \param blockSize Block size (shall be set to 512 in case of high capacity).
 * \param pData     Pointer to the DW aligned buffer to be filled.
 * \param address   Data Address on SD/MMC card.
 * \param pStatus   Pointer to response buffer as status.
 * \param fCallback Pointer to optional callback invoked on command end.
 *                  NULL:    Function return until command finished.
 *                  Pointer: Return immediately and invoke callback at end.
 *                  Callback argument is fixed to a pointer to sSdCard instance.
 */
static inline uint8_t Cmd24(sSdCard *pSd,
							uint8_t *pData,
							uint32_t address,
							uint32_t *pStatus,
							fSdmmcCallback callback)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd24()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATATX(1);
	pCmd->bCmd       = 24;
	pCmd->dwArg      = address;
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = BLOCK_SIZE(pSd);
	pCmd->wNbBlocks  = 1;
	pCmd->pData      = pData;
	pCmd->fCallback  = callback;
	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}


/**
 * Write multiple block command
 * \param pSd  Pointer to a SD card driver instance.
 * \param blockSize Block size (shall be set to 512 in case of high capacity).
 * \param nbBlock   Number of blocks to send.
 * \param pData     Pointer to the DW aligned buffer to be filled.
 * \param address   Data Address on SD/MMC card.
 * \param pStatus   Pointer to the response buffer as status.
 * \param fCallback Pointer to optional callback invoked on command end.
 *                  NULL:    Function return until command finished.
 *                  Pointer: Return immediately and invoke callback at end.
 *                  Callback argument is fixed to a pointer to sSdCard instance.
 */
static uint8_t Cmd25(sSdCard *pSd,
					 uint16_t nbBlock,
					 uint8_t *pData,
					 uint32_t address,
					 uint32_t *pStatus,
					 fSdmmcCallback callback)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd25()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATATX(1);
	pCmd->bCmd       = 25;
	pCmd->dwArg      = address;
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = BLOCK_SIZE(pSd);
	pCmd->wNbBlocks  = nbBlock;
	pCmd->pData      = pData;
	pCmd->fCallback  = callback;
	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * SDIO IO_RW_DIRECT command, response R5.
 * \return the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param pIoData   Pointer to input argument (\ref SdioRwDirectArg) and
 *                  response (\ref SdmmcR5) buffer.
 * \param fCallback Pointer to optional callback invoked on command end.
 *                  NULL:    Function return until command finished.
 *                  Pointer: Return immediately and invoke callback at end.
 *                  Callback argument is fixed to a pointer to sSdCard instance.
 */
static uint8_t Cmd52(sSdCard *pSd,
					 uint8_t wrFlag,
					 uint8_t funcNb,
					 uint8_t rdAfterWr,
					 uint32_t addr,
					 uint32_t *pIoData)
{
	SdioCmd52Arg *pArg52 = (SdioCmd52Arg *)pIoData;
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	pArg52->rwFlag = wrFlag;
	pArg52->functionNum = funcNb;
	pArg52->rawFlag = rdAfterWr;
	pArg52->regAddress = addr;

	TRACE_DEBUG("Cmd52()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(5);
	pCmd->bCmd       = 52;
	pCmd->dwArg      = *pIoData;
	pCmd->pResp      = pIoData;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * SDIO IO_RW_EXTENDED command, response R5.
 * \param pSd       Pointer to a SD card driver instance.
 * \param pArgResp  Pointer to input argument (\ref SdioRwExtArg)
 *                  and response (\ref SdmmcR5) buffer.
 * \param pData     Pointer to data buffer to transfer.
 * \param size      Transfer data size.
 * \param fCallback Pointer to optional callback invoked on command end.
 *                  NULL:    Function return until command finished.
 *                  Pointer: Return immediately and invoke callback at end.
 *                  Callback argument is fixed to a pointer to sSdCard instance.
 * \param pArg Callback argument.
 */
static uint8_t Cmd53(sSdCard *pSd,
					 uint8_t wrFlag,
					 uint8_t funcNb,
					 uint8_t blockMode,
					 uint8_t incAddr,
					 uint32_t addr,
					 uint8_t *pIoData,
					 uint16_t len,
					 uint32_t *pArgResp,
					 fSdmmcCallback fCallback,
					 void *pCbArg)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	SdioCmd53Arg *pArg53;
	pArg53 = (SdioCmd53Arg *)pArgResp;
	pArg53->rwFlag = wrFlag;
	pArg53->functionNum = funcNb;
	pArg53->blockMode = blockMode;
	pArg53->opCode = incAddr;
	pArg53->regAddress = addr;
	pArg53->count = len;

	/* Fill command */
	pCmd->bCmd       = 53;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(5)
					   | SDMMC_CMD_bmIO;

	if (wrFlag)
		pCmd->cmdOp.wVal |= SDMMC_CMD_bmDATATX;
	else
		pCmd->cmdOp.wVal |= SDMMC_CMD_bmDATARX;

	if (blockMode)
		pCmd->wBlockSize = pSd->wCurrBlockLen;
	else {
		/* Force byte mode */
		pCmd->wBlockSize = 0;
		//pCmd->wBlockSize = 1;
	}

	pCmd->dwArg     = *pArgResp;
	pCmd->pResp     = pArgResp;
	pCmd->pData     = pIoData;
	pCmd->wNbBlocks = len;
	pCmd->fCallback = fCallback;
	pCmd->pArg      = pCbArg;

	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Indicates to the card that the next command is an application specific
 * command rather than a standard command.
 * \return the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param cardAddr  Card Relative Address.
 */
static uint8_t Cmd55(sSdCard *pSd, uint16_t cardAddr)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint32_t dwResp;
	uint8_t  bRc;

	TRACE_DEBUG("Cmd55()\r\n");
	_ResetCmd(pCmd);

	/* Fill command information */
	pCmd->bCmd       = 55;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(1) | (cardAddr ? 0 : SDMMC_CMD_bmOD);
	pCmd->dwArg      = cardAddr << 16;
	pCmd->pResp      = &dwResp;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Sends the status register.
 * Should be invoked after Cmd55().
 * Returns the command transfer result (see SendMciCommand).
 * \param pSd       Pointer to a SD card driver instance.
 * \param pStatus   Pointer to a status variable.
 */
static uint8_t SdACmd13(sSdCard *pSd, uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("ACmd13()\r\n");
	_ResetCmd(pCmd);

	/* Fill command */
	pCmd->bCmd       = 13;
	pCmd->cmdOp.wVal = SDMMC_CMD_CDATARX(1);
	pCmd->pResp      = pStatus;
	pCmd->wBlockSize = 512 / 8;
	pCmd->wNbBlocks  = 1;
	pCmd->pData      = (uint8_t *)pSd->SSR;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * To get the SSR register of SD card.
 * \param pSd     Pointer to a SD card driver instance.
 * \param pStatus Pointer to buffer for command response as status.
 * \return the command transfer result (see SendMciCommand).
 */
static uint8_t Acmd13(sSdCard *pSd, uint32_t *pStatus)
{
	uint8_t error;
	uint8_t  bRc;

	error = Cmd55(pSd, CARD_ADDR(pSd));

	if (error) {
		TRACE_ERROR("Acmd13.cmd55:%d\r\n", error);
		return error;
	}

	bRc = SdACmd13(pSd, pStatus);
	return bRc;
}

/**
 * Defines the data bus width (00=1bit or 10=4 bits bus) to be used for data
 * transfer.
 * The allowed data bus widths are given in SCR register.
 * Should be invoked after SdmmcCmd55().
 * \param pSd     Pointer to a SD card driver instance.
 * \param arg     Argument for this command.
 * \param pStatus Pointer to buffer for command response as status.
 * \return the command transfer result (see SendMciCommand).
 */
static uint8_t SdAcmd6(sSdCard *pSd,
					   uint32_t arg,
					   uint32_t *pStatus)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Acmd6()\r\n");
	_ResetCmd(pCmd);

	/* Fill command information */
	pCmd->bCmd       = 6;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(1);
	pCmd->pResp      = pStatus;
	pCmd->dwArg      = arg;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Defines the data bus width (00=1bit or 10=4 bits bus) to be used for data
 * transfer.
 * The allowed data bus widths are given in SCR register.
 * \param pSd  Pointer to a SD card driver instance.
 * \param busWidth  Bus width in bits (4 or 1).
 * \return the command transfer result (see SendCommand).
 */
static uint8_t Acmd6(sSdCard *pSd, uint8_t busWidth)
{
	uint8_t error;
	uint32_t arg;
	error = Cmd55(pSd, CARD_ADDR(pSd));

	if (error) {
		TRACE_ERROR("Acmd6.cmd55:%d\r\n", error);
		return error;
	}

	arg = (busWidth == 4) ? SD_ST_DATA_BUS_WIDTH_4BIT : SD_ST_DATA_BUS_WIDTH_1BIT;
	return SdAcmd6(pSd, arg, NULL);
}

/**
 * Asks to all cards to send their operations conditions.
 * Returns the command transfer result (see SendMciCommand).
 * Should be invoked after SdmmcCmd55().
 * \param pSd   Pointer to a SD card driver instance.
 * \param pOpIo Pointer to argument that should be sent and OCR content.
 */
static uint8_t SdAcmd41(sSdCard *pSd, uint32_t *pOpIo)
{
	sSdmmcCommand *pCmd = &pSd->sdCmd;
	uint8_t  bRc;

	TRACE_DEBUG("Acmd41()\r\n");
	_ResetCmd(pCmd);

	/* Fill command information */
	pCmd->bCmd       = 41;
	pCmd->cmdOp.wVal = SDMMC_CMD_CNODATA(3);
	pCmd->dwArg      = *pOpIo;
	pCmd->pResp      = pOpIo;

	/* Send command */
	bRc = _SendCmd(pSd, NULL, NULL);
	return bRc;
}

/**
 * Asks to all cards to send their operations conditions.
 * Returns the command transfer result (see SendCommand).
 * \param pSd  Pointer to a SD card driver instance.
 * \param hcs  Shall be true if Host support High capacity.
 * \param pCCS  Set the pointed flag to 1 if hcs != 0 and SD OCR CCS flag is set.
 */
static uint8_t Acmd41(sSdCard *pSd, uint8_t hcs, uint8_t *pCCS)
{
	uint8_t error;
	uint32_t arg;

	do {
		error = Cmd55(pSd, 0);

		if (error) {
			TRACE_ERROR("Acmd41.cmd55:%d\r\n", error);
			return error;
		}

		arg = SD_HOST_VOLTAGE_RANGE;

		if (hcs) arg |= SD_OCR_CCS;

		error = SdAcmd41(pSd, &arg);

		if (error) {
			TRACE_ERROR("Acmd41.cmd41:%d\r\n", error);
			return error;
		}

		*pCCS = ((arg & SD_OCR_CCS) != 0);
	} while ((arg & SD_OCR_BUSY) != SD_OCR_BUSY);

	return 0;
}

/**
 * Try SW Reset several times (CMD0 with ARG 0)
 * \param pSd      Pointer to a SD card driver instance.
 * \param retry    Retry times.
 * \return 0 or MCI error code.
 */
static uint8_t SwReset(sSdCard *pSd, uint32_t retry)
{
	uint32_t i;
	uint8_t error = 0;

	for (i = 0; i < retry; i ++) {
		error = Cmd0(pSd, 0);

		if (error != SDMMC_ERROR_NORESPONSE)
			break;
	}

	return error;
}
/**
 * Stop TX/RX
 */

uint32_t s_max_i = 0;
uint32_t s_max_j = 0;

static uint8_t _StartCmd(sSdCard *pSd)
{
  uint32_t status;
	uint32_t state;
  volatile uint32_t j;
  uint8_t error = 0;
  
	for (j = 0; j < 5000; j ++)
    {
      s_max_j = max(s_max_j, j);
      error = Cmd13(pSd, &status);

			if (error) {
				TRACE_ERROR("_StopC.Cmd13: st%x, er%d\r\n", pSd->bState, error);
				return error;
			}

			state = status & STATUS_STATE;

			/* Invalid state */
			if (state == STATUS_IDLE
					|| state == STATUS_READY
					|| state == STATUS_IDENT
					|| state == STATUS_STBY
					|| state == STATUS_DIS) {
				TRACE_ERROR("_StopC.state\r\n");
				return SDMMC_ERROR_NOT_INITIALIZED;
			}

			/* Ready? */
			if ((status & STATUS_READY_FOR_DATA) == STATUS_READY_FOR_DATA
				   &&  state == STATUS_TRAN)
				return SDMMC_SUCCESS;
        
        if(j == 2)
          _StopCmd(pSd);
        
        taskYIELD();
		}  
    return 0;
}

static uint8_t _StopCmd(sSdCard *pSd)
{
	uint32_t status;
	uint32_t state;
	volatile uint32_t i, j, k;
	uint8_t error = 0;

	/* Retry stop for 9 times */
	for (i = 0; i < 9; i ++)
  {
    s_max_i = max(s_max_i, i);
    error = Cmd12(pSd, &status);

    if (error) {
			TRACE_ERROR("_StopC.Cmd12: st%x, er%d\r\n", pSd->bState, error);
			return error;
		}
//return SDMMC_SUCCESS;

		/* Check status for 299 times */
		for (j = 0; j < 299; j ++)
    {
      s_max_j = max(s_max_j, j);
      for(k = 0; k<1000; k++);
      error = Cmd13(pSd, &status);

			if (error) {
				TRACE_ERROR("_StopC.Cmd13: st%x, er%d\r\n", pSd->bState, error);
				return error;
			}

			state = status & STATUS_STATE;

			/* Invalid state */
			if (state == STATUS_IDLE
					|| state == STATUS_READY
					|| state == STATUS_IDENT
					|| state == STATUS_STBY
					|| state == STATUS_DIS) {
				TRACE_ERROR("_StopC.state\r\n");
				return SDMMC_ERROR_NOT_INITIALIZED;
			}

			/* Ready? */
			if ((status & STATUS_READY_FOR_DATA) == STATUS_READY_FOR_DATA
				   &&  state == STATUS_TRAN)
				return SDMMC_SUCCESS;

		}

		/* For write, try stop command again */
		if (state != STATUS_RCV)
			break;
	}

	return SDMMC_ERROR_STATE;
}


/**
 * Perform sligle block transfer
 */
static uint8_t PerformSingleTransfer(sSdCard *pSd,
									 uint32_t address,
									 uint8_t *pData,
									 uint8_t isRead)
{
	uint32_t status;
	uint8_t error = 0;
	uint32_t sdmmc_address;

  sdmmc_address = SD_ADDRESS(pSd, address);

if(!pSd->bCmd23)
{
		if ((pSd->bState == SDMMC_STATE_DATA_RD)			|| (pSd->bState == SDMMC_STATE_DATA_WR)) {
			error = _StopCmd(pSd);
			if (error) return error;
			//pSd->bState = SDMMC_STATE_TRAN;
			//pSd->dwPrevBlk = 0xFFFFFFFF;
		
	}
}  


	if (isRead) {
		/* Read single block */
		error = Cmd17(pSd, pData, sdmmc_address, &status, NULL);

		if (error) {
			TRACE_ERROR("SingleTx.Cmd17: %d\r\n", error);
			return error;
		}

		if (status & ~(STATUS_READY_FOR_DATA | STATUS_STATE)) {
			TRACE_ERROR("CMD17.stat: %lx\r\n",
						status & ~(STATUS_READY_FOR_DATA | STATUS_STATE));
			return SDMMC_ERROR;
		}

		return error;
	}

	/* Write */
	{
		/* Move to Sending data state */
		error = Cmd24(pSd,
					  pData, sdmmc_address,
					  &status,
					  NULL);

		if (error) {
			TRACE_DEBUG("SingleTx.Cmd24: %d\r\n", error);
			return error;
		}

		if (status & (STATUS_WRITE & ~(STATUS_READY_FOR_DATA | STATUS_STATE))) {
			TRACE_ERROR("CMD24(0x%x).stat: %x\r\n",
						(unsigned int) sdmmc_address,
						(unsigned int)(status & (STATUS_WRITE & ~(STATUS_READY_FOR_DATA
												 | STATUS_STATE))));
			return SDMMC_ERROR;
		}
	}

	return error;
}

/**
 * Move SD card to transfer state. The buffer size must be at
 * least 512 byte long. This function checks the SD card status register and
 * address the card if required before sending the transfer command.
 * Returns 0 if successful; otherwise returns an code describing the error.
 * \param pSd      Pointer to a SD card driver instance.
 * \param address  Address of the block to transfer.
 * \param nbBlocks Number of blocks to be transfer, 0 for infinite transfer.
 * \param pData    Data buffer whose size is at least the block size.
 * \param isRead   1 for read data and 0 for write data.
 */
static uint8_t MoveToTransferState(sSdCard *pSd,
								   uint32_t address,
								   uint16_t nbBlocks,
								   uint8_t *pData,
								   uint8_t isRead)
{
	uint32_t status;
	uint8_t error;
	uint32_t sdmmc_address;

  sdmmc_address = SD_ADDRESS(pSd, address);

  /* Defines the number of blocks read for a block read command */
  if(pSd->bCmd23)
  {
    error = Cmd23(pSd, 0, nbBlocks, &status);
    if (error) {
      TRACE_ERROR("SetBlockCount.Cmd23: %hhu\r\n", error);
      return error;
    }
  }
  else
  {
#ifdef STOPINCB
    _StartCmd(pSd);
#else
    if( (pSd->bState == SDMMC_STATE_DATA_RD) || (pSd->bState == SDMMC_STATE_DATA_WR) )
    {
      error = _StopCmd(pSd);
      if (error) return error;
    }
#endif
  }

	if(isRead)
  {
		/* Move to Receiving data state */
		error = Cmd18(pSd, nbBlocks, pData, sdmmc_address, &status, NULL);

		if (error) {
			TRACE_ERROR("MTranState.Cmd18: %hhu\r\n", error);
			return error;
		}

		if (status & ~(STATUS_READY_FOR_DATA | STATUS_STATE))
    {
			TRACE_ERROR("CMD18.stat: %lx\r\n", status & ~(STATUS_READY_FOR_DATA | STATUS_STATE));
			return SDMMC_ERROR;
		}
	}
  else
  {
		/* Move to Sending data state */
		error = Cmd25(pSd, nbBlocks, pData, sdmmc_address, &status, NULL);

		if (error) {
			TRACE_DEBUG("MoveToTransferState.Cmd25: %huu\r\n", error);
			return error;
		}

		if (status & (STATUS_WRITE & ~(STATUS_READY_FOR_DATA | STATUS_STATE)))
    {
			TRACE_ERROR("CMD25(0x%lx, %hu).stat: %lx\r\n", sdmmc_address, nbBlocks, (status & (STATUS_WRITE & ~(STATUS_READY_FOR_DATA | STATUS_STATE))));
			return SDMMC_ERROR;
		}
	}

	return error;
}

/**
 * Switch card state between STBY and TRAN (or CMD and TRAN)
 * \param pSd       Pointer to a SD card driver instance.
 * \param address   Card address to TRAN, 0 to STBY
 * \param statCheck Whether to check the status before CMD7.
 */
static uint8_t MmcSelectCard(sSdCard *pSd, uint16_t address, uint8_t statCheck)
{
	uint8_t error;
	uint32_t  status;
	uint32_t  targetState = address ? STATUS_TRAN : STATUS_STBY;
	uint32_t  srcState    = address ? STATUS_STBY : STATUS_TRAN;

	/* At this stage the Initialization and identification process is achieved
	 * The SD card is supposed to be in Stand-by State */
	while (statCheck) {
		error = Cmd13(pSd, &status);

		if (error) {
			TRACE_ERROR("MmcSelectCard.Cmd13 (%d)\r\n", error);
			return error;
		}

		if ((status & STATUS_READY_FOR_DATA)) {
			uint32_t currState = status & STATUS_STATE;

			if (currState == targetState) return 0;

			if (currState != srcState) {
				TRACE_ERROR("MmcSelectCard, wrong state %x\r\n",
							(unsigned int)currState);
				return SDMMC_ERROR;
			}

			break;
		}
	}

	/* witch to TRAN mode to Select the current SD/MMC
	 * so that SD ACMD6 can process or EXT_CSD can read. */
	error = Cmd7(pSd, address);

	if (error == SDMMC_ERROR_NOT_INITIALIZED && address == 0) {}
	else if (error)
		TRACE_ERROR("MmcSelectCard.Cmd7 (%d)\r\n", error);

	return error;
}

/**
 * Get SD/MMC memory max transfer speed in K.
 * \param pSd Pointer to a SD card driver instance.
 */
static uint32_t SdmmcGetMaxSpeed(sSdCard *pSd)
{
	uint32_t speed = 0;
	TRACE_INFO(" .TRAN_SPEED         0x%X\r\n",
			   (unsigned int)SD_CSD_TRAN_SPEED(pSd->CSD));


		speed = SdmmcDecodeTransSpeed(SD_CSD_TRAN_SPEED(pSd->CSD),
									  sdmmcTransUnits,
									  sdTransMultipliers);

	return speed;
}

/**
 * Get SDIO max transfer speed, in K.
 * \param pSd Pointer to a SD card driver instance.
 */
static uint32_t SdioGetMaxSpeed(sSdCard *pSd)
{
	uint8_t error;
	uint32_t speed = 0;
	uint32_t addr = 0;
	uint8_t  buf[6];

	if (pSd->bCardType & CARD_TYPE_bmSDIO) {
		/* Check Func0 tuple in CIS area */
		error = SDIO_ReadDirect(pSd,
								SDIO_CIA, SDIO_CIS_PTR_REG,
								(uint8_t *)&addr, 3);

		if (error) {
			TRACE_ERROR("SdioUpdateCardInformation.RdDirect: %d\r\n", error);
			return error;
		}

		error = SdioFindTuples(pSd, addr, 256, NULL, &addr);

		if (error) {
			TRACE_ERROR("SdioUpdateCardInformation.FindTuple: %d\r\n", error);
			return error;
		}

		if (addr) {
			/* Fun0 tuple: fn0_blk_siz & max_tran_speed */
			SDIO_ReadDirect(pSd, SDIO_CIA, addr, buf, 6);
			speed = SdmmcDecodeTransSpeed(buf[5],
										  sdmmcTransUnits,
										  sdTransMultipliers);
		}
	}

	return speed;
}

/**
 * Get SD Status information
 * \param pSd      Pointer to a SD card driver instance.
 */
static void SdGetExtInformation(sSdCard *pSd)
{
	uint8_t  error;
	uint32_t pStatus;
	error = MmcSelectCard(pSd, CARD_ADDR(pSd), 1);

	if (error) {
		TRACE_ERROR("SdMmcInit.SelCard(%d)\r\n", error);
		return;
	}

	Acmd13(pSd, &pStatus);
	
  
  error = MmcSelectCard(pSd, CARD_ADDR(pSd), 1);

	if (error) {
		TRACE_ERROR("SdMmcInit.SelCard(%d)\r\n", error);
		return;
	}
  
  Acmd51(pSd);
}

/**
 * Update SD/MMC information.
 * Update CSD for card speed switch.
 * Update ExtDATA for any card function switch.
 * \param pSd      Pointer to a SD card driver instance.
 * \return error code when update CSD error.
 */
static void SdMmcUpdateInformation(sSdCard *pSd,
								   uint8_t csd,
								   uint8_t extData)
{
	uint8_t error;

	/* Update CSD for new TRAN_SPEED value */
	if (csd) {
		MmcSelectCard(pSd, 0, 1);
		vTaskDelay(800);
    error = Cmd9(pSd);

		if (error) {
			TRACE_ERROR("SdMmcUpdateInfo.Cmd9 (%d)\r\n", error);
			return;
		}

		error = MmcSelectCard(pSd, pSd->wAddress, 1);
	}

	if (extData) {
		switch (pSd->bCardType & CARD_TYPE_bmSDMMC) {
		case CARD_TYPE_bmSD:    SdGetExtInformation(pSd);   break;

		default: break;
		}
	}
}

/**
 * \brief Check HS capability and enable it
 * \param pSd Pointer to sSdCard instance.
 */
static uint8_t SdMmcEnableHighSpeed(sSdCard *pSd)
{
	uint8_t  error;
	uint8_t sd;
	uint32_t status;
	sd  = ((pSd->bCardType & CARD_TYPE_bmSDMMC) == CARD_TYPE_bmSD);

	/* Check host driver capability */
	if (_HwGetHsMode(pSd) == 0)
  {
		return SDMMC_ERROR_NOT_SUPPORT;
  }    

		if (sd)
    {
      
      pSd->bCmd23 = SD_SCR_SD_CMD23(pSd->SCR);
      
      printf("SD_SPEC: %lu\r\n", SD_SCR_SD_SPEC(pSd->SCR));
      printf("SD_SPEC3: %lu\r\n", SD_SCR_SD_SPEC3(pSd->SCR));
      printf("SD_SPEC4: %lu\r\n", SD_SCR_SD_SPEC4(pSd->SCR));
      printf("SD_SPECX: %lu\r\n", SD_SCR_SD_SPECX(pSd->SCR));
      printf("CMD23 supported: %hhu\r\n", pSd->bCmd23);
      
			/* Check version */
			if (SD_CSD_STRUCTURE(pSd->CSD) >= 1) {
				/* Try enable HS mode */
				SdCmd6Arg cmd6Arg = {
					1, 0, 0xF, 0xF, 0xF, 0xF, 0, 1
				};
				//uint32_t switchStatus[512/32];
				error = SdCmd6(pSd, &cmd6Arg, switchStatus, &status);

				if (error || (status & STATUS_SWITCH_ERROR)) {
					TRACE_ERROR("SD HS Fail\r\n");
					return SDMMC_ERROR;
				} else if (SD_SWITCH_ST_FUN_GRP1_RC(switchStatus)
						   == SD_SWITCH_ST_FUN_GRP_RC_ERROR) {
					TRACE_WARNING("SD HS Not Supported\r\n");
					return SDMMC_ERROR_NOT_SUPPORT;
				} else if (SD_SWITCH_ST_FUN_GRP1_BUSY(switchStatus)) {
					TRACE_ERROR("SD HS Locked\r\n");
					return SDMMC_ERROR_BUSY;
				} else
					TRACE_INFO("SD HS Enabled\r\n");
			} else {
				TRACE_INFO("HS Not Supported in SD Rev 0x%x\r\n",
						   (unsigned)SD_SCR_SD_SPEC(pSd));
				return SDMMC_ERROR_NOT_SUPPORT;
			}
		}

	/* Enable Host HS mode */
	_HwSetHsMode(pSd, 1);
	return 0;
}

COMPILER_ALIGNED(32) const uint8_t data_8bits[8] = {0xaa, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
COMPILER_ALIGNED(32) const uint8_t data_4bits[4] = {0xa5, 0x00, 0x00, 0x00};

/**
 * \brief Check bus width capability and enable it
 */
static uint8_t SdMmcDesideBuswidth(sSdCard *pSd)
{
	uint8_t  error, busWidth;


	/* Best width that the card support */
	busWidth = 4;

		/* SD(IO): switch to 4-bit mode ? */
		uint8_t io  = ((pSd->bCardType & CARD_TYPE_bmSDIO) > 0);
		uint8_t sd  = ((pSd->bCardType & CARD_TYPE_bmSDMMC) == CARD_TYPE_bmSD);

		if (busWidth == 1)
			return SDMMC_ERROR_NOT_SUPPORT;

		/* No 8-bit mode, default to 4-bit mode */
		busWidth = 4;

		/* SDIO */
		if (io) {
			/* SDIO 1 bit only */
			busWidth = 1;
		}

		/* SD */
		if (sd) {
			error = Acmd6(pSd, busWidth);

			if (error) {
				TRACE_ERROR("SdMmcSetBusWidth.Acmd6: %d\r\n", error);
				return SDMMC_ERROR;
			}

			TRACE_WARNING("SD 4-bit mode\r\n");
		}

		/* Switch to selected bus mode */
		pSd->bBusMode = busWidth;
		_HwSetBusWidth(pSd, busWidth);


	return 0;
}

/**
 * \brief Run the SD/MMC/SDIO Mode initialization sequence.
 * This function runs the initialization procedure and the identification
 * process. Then it leaves the card in ready state. The following procedure must
 * check the card type and continue to put the card into tran(for memory card)
 * or cmd(for io card) state for data exchange.
 * \param pSd  Pointer to a SD card driver instance.
 * \return 0 if successful; otherwise returns an \ref sdmmc_rc "SD_ERROR code".
 */
static uint8_t SdMmcIdentify(sSdCard *pSd)
{
	uint8_t mem = 0, io = 0, f8 = 0, mp = 1, ccs = 0;
	uint32_t status;
	uint8_t error;
	/* Reset HC to default HS and BusMode */
	_HwSetHsMode(pSd, 0);
	_HwSetBusWidth(pSd, 1);
	/* Reset SDIO: CMD52, write 1 to RES bit in CCCR (bit 3 of register 6) */
	status = SDIO_RES;
	error = Cmd52(pSd, 1, SDIO_CIA, 0, SDIO_IOA_REG, &status);

	if (!error && ((status & STATUS_SDIO_R5) == 0)) {}
	else if (error == SDMMC_ERROR_NORESPONSE) {}
	else
		TRACE_DEBUG("SdMmcIdentify.Cmd52 fail: %u, %lx\r\n", error, status);

	/* Reset MEM: CMD0 */
	error = SwReset(pSd, 1);

	if (error)
		TRACE_DEBUG("SdMmcIdentify.SwReset fail: %u\r\n", error);

	/* CMD8 is newly added in the Physical Layer Specification Version 2.00 to
	 * support multiple voltage ranges and used to check whether the card
	 * supports supplied voltage. The version 2.00 host shall issue CMD8 and
	 * verify voltage before card initialization.
	 * The host that does not support CMD8 shall supply high voltage range... */
	error = SdCmd8(pSd, 1);

	if (error == 0) f8 = 1;
	else if (error != SDMMC_ERROR_NORESPONSE) {
		TRACE_ERROR("SdMmcIdentify.Cmd8: %d\r\n", error);
		return SDMMC_ERROR;
	}
	/* Delay after "no response" */
	else
  {
     vTaskDelay(250);
  }
	
  /* CMD5 is newly added for SDIO initialize & power on */
	status = 0;
	error = Cmd5(pSd, &status);

	if (error)
  {
		TRACE_INFO("SdMmcIdentify.Cmd5: %d\r\n", error)
	}
	/* Card has SDIO function */
	else if ((status & SDIO_OCR_NF) > 0)
  {
    unsigned int cmd5Retries = 10000;
		do
    {
			status &= SD_HOST_VOLTAGE_RANGE;
			error = Cmd5(pSd, &status);

			if (status & SD_OCR_BUSY) break;
		} while (!error && cmd5Retries --);

		if (error)
    {
			TRACE_ERROR("SdMmcIdentify.Cmd5 V: %d\r\n", error);
			return SDMMC_ERROR;
		}

		io = 1;
		TRACE_INFO("SDIO\r\n");
    
		/* IO only ?*/
		mp = ((status & SDIO_OCR_MP) > 0);
	}

	/* Has memory: SD/MMC/COMBO */
	if (mp) {
		/* Try SD memory initialize */
		error = Acmd41(pSd, f8, &ccs);

		if (error)
    {
			TRACE_DEBUG("SdMmcIdentify.Acmd41: %u, try MMC\r\n", error);
	  	return SDMMC_ERROR;
		}
    else if (ccs)
    {
			TRACE_INFO("SDHC MEM\r\n");
		}
		else
    {
			TRACE_INFO("SD MEM\r\n");
		}

		mem = 1;
	}

	/* SD(IO) + MEM ? */
	if (!mem) {
		if (io) pSd->bCardType = CARD_SDIO;
		else {
			TRACE_ERROR("Unknown card\r\n");
			return SDMMC_ERROR;
		}
	}
	/* SD(HC) combo */
	else if (io)
		pSd->bCardType = ccs ? CARD_SDHCCOMBO : CARD_SDCOMBO;
	/* SD(HC) */
	else
		pSd->bCardType = ccs ? CARD_SDHC : CARD_SD;

	return 0;
}

/**
 * \brief Run the SD/MMC/SDIO enumeration sequence.
 * This function runs after the initialization and identification procedure. It
 * gets all necessary information from the card and deside transfer block size,
 * clock speed and bus width. It sets the SD/MMC/SDIO card in transfer
 * (or command) state.
 * \param pSd  Pointer to a SD card driver instance.
 * \return 0 if successful; otherwise returns an \ref sdmmc_rc "SD_ERROR code".
 */
static uint8_t SdMmcEnum(sSdCard *pSd)
{
	uint8_t mem , io;
	uint8_t error;
	uint32_t ioSpeed = 0, memSpeed = 0;
	uint8_t hsExec = 0, bwExec = 0;

	/* - has Memory/IO/High-Capacigy - */
	mem = ((pSd->bCardType & CARD_TYPE_bmSDMMC) > 0);
	io  = ((pSd->bCardType & CARD_TYPE_bmSDIO)  > 0);

	/* For MEMORY cards:
	 * The host then issues the command ALL_SEND_CID (CMD2) to the card to get
	 * its unique card identification (CID) number.
	 * Card that is unidentified (i.e. which is in Ready State) sends its CID
	 * number as the response (on the CMD line). */
	if (mem) {
		error = Cmd2(pSd);

		if (error) {
			TRACE_ERROR("SdMmcInit.cmd2(%d)\r\n", error);
			return error;
		}
	}

	/* For MEMORY and SDIO cards:
	 * Thereafter, the host issues CMD3 (SEND_RELATIVE_ADDR) asks the
	 * card to publish a new relative card address (RCA), which is shorter than
	 * CID and which is used to address the card in the future data transfer
	 * mode. Once the RCA is received the card state changes to the Stand-by
	 * State. At this point, if the host wants to assign another RCA number, it
	 * can ask the card to publish a new number by sending another CMD3 command
	 * to the card. The last published RCA is the actual RCA number of the
	 * card. */
	error = Cmd3(pSd);

	if (error) {
		TRACE_ERROR("SdMmcInit.cmd3(%d)\r\n", error);
		return error;
	}

	/* For MEMORY cards:
	 * SEND_CSD (CMD9) to obtain the Card Specific Data (CSD register),
	 * e.g. block length, card storage capacity, etc... */
	if (mem) {
		error = Cmd9(pSd);

		if (error) {
			TRACE_ERROR("SdMmcInit.cmd9(%d)\r\n", error);
			return error;
		}
	}

	/* Now select the card, to TRAN state */
	error = MmcSelectCard(pSd, CARD_ADDR(pSd), 0);

	if (error) {
		TRACE_ERROR("SdMmcInit.SelCard(%d)\r\n", error);
		return error;
	}

	/* - Now in TRAN, obtain extended setup information - */

	/* If the card support EXT_CSD, read it! */
	TRACE_INFO("Card Type %u, CSD_STRUCTURE %u\r\n",
			   (unsigned int)pSd->bCardType, (unsigned int)SD_CSD_STRUCTURE(pSd->CSD));

	/* Get extended information of the card */
	SdMmcUpdateInformation(pSd, 1, 1);

	/* Calculate transfer speed */
//	if (io)     ioSpeed = SdioGetMaxSpeed(pSd);

	if (mem)    memSpeed = SdmmcGetMaxSpeed(pSd);


	/* Combo, min speed */
	if (io && mem)
		pSd->dwTranSpeed = (ioSpeed > memSpeed) ? memSpeed : ioSpeed;
	/* SDIO only */
	else if (io)
		pSd->dwTranSpeed = ioSpeed;
	/* Memory card only */
	else if (mem)
		pSd->dwTranSpeed = memSpeed;

	pSd->dwTranSpeed *= 1000;

	/* Enable more bus width Mode */
	error = SdMmcDesideBuswidth(pSd);

	if (!error) bwExec = 1;
	else if (error != SDMMC_ERROR_NOT_SUPPORT) {
		TRACE_ERROR("SdmmcEnum.DesideBusWidth: %d\r\n", error);
		return SDMMC_ERROR;
	}

	/* Enable High-Speed Mode */
	error = SdMmcEnableHighSpeed(pSd);

	if (!error) hsExec = 1;
	else if (error != SDMMC_ERROR_NOT_SUPPORT)
		return SDMMC_ERROR;

	/* In HS mode transfer speed *2 */
	if (hsExec) pSd->dwTranSpeed *= 2;

	/* Update card information since status changed */
	if (bwExec || hsExec) SdMmcUpdateInformation(pSd, hsExec, 0);

	return 0;
}


/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/** \brief Dump register.
 */
void _DumpREG(void *pREG, uint32_t dwSize)
{
	uint8_t *p = (uint8_t *)pREG;
	uint32_t i;

	for (i = 0; i < dwSize; i ++) {
		if ((i % 16) == 0) printf("\r\n [%04X]", (unsigned int)i);

		printf(" %02X", p[i]);
	}

	printf("\r\n");
}


/**
 * Read Blocks of data in a buffer pointed by pData. The buffer size must be at
 * least 512 byte long. This function checks the SD card status register and
 * address the card if required before sending the read command.
 * \return 0 if successful; otherwise returns an \ref sdmmc_rc "error code".
 * \param pSd      Pointer to a SD card driver instance.
 * \param address  Address of the block to read.
 * \param pData    Data buffer whose size is at least the block size, it can
 *            be 1,2 or 4-bytes aligned when used with DMA.
 * \param length   Number of blocks to be read.
 * \param pCallback Pointer to callback function that invoked when read done.
 *                  0 to start a blocked read.
 * \param pArgs     Pointer to callback function arguments.
 */
uint8_t SD_Read(sSdCard        *pSd,
				uint32_t      address,
				void          *pData,
				uint32_t      length,
				fSdmmcCallback pCallback,
				void          *pArgs)
{
	assert(pSd != NULL);
	assert(pData != NULL);
    /*dummy*/
    pCallback = pCallback;
    pArgs = pArgs;
    
	pSd->bState = SDMMC_STATE_DATA_RD;
	TRACE_DEBUG("MMCT_ReadFun(pSd,0x%lx,%ld,pBuffer); \r\n", address, length);
	TRACE_DEBUG("R %lx,%lu ", address, length);
  if(length > 1)
  {
	  MoveToTransferState(pSd, address, length, (uint8_t *)pData, 1);
  }
  else
  {
    PerformSingleTransfer(pSd, address, (uint8_t *)pData, 1);
  }      
	TRACE_DEBUG("SDrd(%lx,%lu)\r\n", address, length);
	return 0;
}

/**
 * Write Blocks of data in a buffer pointed by pData. The buffer size must be at
 * least 512 byte long. This function checks the SD card status register and
 * address the card if required before sending the read command.
 * \return 0 if successful; otherwise returns an \ref sdmmc_rc "error code".
 * \param pSd      Pointer to a SD card driver instance.
 * \param address  Address of the block to write.
 * \param pData    Data buffer whose size is at least the block size, it can
 *            be 1,2 or 4-bytes aligned when used with DMA.
 * \param length   Number of blocks to be write.
 * \param pCallback Pointer to callback function that invoked when write done.
 *                  0 to start a blocked write.
 * \param pArgs     Pointer to callback function arguments.
 */
uint8_t SD_Write(sSdCard       *pSd,
				 uint32_t      address,
				 void          *pData,
				 uint32_t      length,
				 fSdmmcCallback pCallback,
				 void          *pArgs)
{
	assert(pSd != NULL);
    /*dummy*/
    pCallback = pCallback;
    pArgs = pArgs;

	pSd->bState = SDMMC_STATE_DATA_WR;
	TRACE_DEBUG("W %lx,%lu ", address, length);
	if(length > 1)
  {
    MoveToTransferState(pSd, address, length, (uint8_t *)pData, 0);
  }
  else
  {
    PerformSingleTransfer(pSd, address, pData, 0);
  }        
	TRACE_DEBUG("SDwr(%lx,%lu)\r\n", address, length);
	return 0;
}


/**
 * Reset SD/MMC driver runtime parameters.
 */
static void _SdParamReset(sSdCard *pSd)
{
	uint32_t i;

	pSd->dwTranSpeed = 0;
	pSd->dwTotalSize = 0;
	pSd->dwNbBlocks  = 0;
	pSd->wBlockSize  = 0;

	pSd->wCurrBlockLen = 0;
	pSd->dwCurrSpeed   = 0;
	pSd->wAddress      = 0;

	pSd->bCardType     = 0;
	pSd->bStatus       = 0;
	pSd->bState        = SDMMC_STATE_IDENT;


	/* Clear CID, CSD, EXT_CSD data */
	for (i = 0; i < 128 / 8 / 4; i ++)  pSd->CID[i] = 0;

	for (i = 0; i < 128 / 8 / 4; i ++)  pSd->CSD[i] = 0;

	for (i = 0; i < 512 / 4; i ++)    pSd->EXT[i] = 0;

	for (i = 0; i < 512 / 8 / 4; i ++)    pSd->SSR[i] = 0;

  memset(pSd->CSR, 0, sizeof(pSd->CSR));
  memset(pSd->SCR, 0, sizeof(pSd->SCR));

}

/**
 * Initialize SD/MMC driver structure.
 * \param pSd   Pointer to a SD card driver instance.
 * \param pDrv  Pointer to low level driver instance.
 * \param bSlot Slot number.
 * \param pHalf Pointer to hardware access functions.
 */
void SDD_Initialize(sSdCard *pSd,
					const void *pDrv, uint8_t bSlot)
{
	pSd->pDrv  = (void *)pDrv;
	pSd->pExt  = NULL;
	pSd->bSlot = bSlot;

	_SdParamReset(pSd);
}

/**
 * Run the SDcard initialization sequence. This function runs the
 * initialisation procedure and the identification process, then it sets the
 * SD card in transfer state to set the block length and the bus width.
 * \return 0 if successful; otherwise returns an \ref sdmmc_rc "error code".
 * \param pSd  Pointer to a SD card driver instance.
 */
uint8_t SD_Init(sSdCard *pSd)
{
	uint8_t  error;
	uint32_t clock;

  //sdMutex = xSemaphoreCreateMutexStatic(&sdMutexBuffer);
  //xSemaphoreTake(sdMutex, portMAX_DELAY);

  sdSemaphore = xSemaphoreCreateBinaryStatic(&sdSemaphoreBuffer);

	_SdParamReset(pSd);

	/* Set low speed for device identification (LS device max speed) */
	clock = 400000;
	_HwSetClock(pSd, &clock);

	/* Initialization delay: The maximum of 1 msec, 74 clock cycles and supply
	 * ramp up time. Supply ramp up time provides the time that the power is
	 * built up to the operating level (the bus master supply voltage) and the
	 * time to wait until the SD card can accept the first command. */
	/* Power On Init Special Command */
	error = Pon(pSd);

	if (error) {
		TRACE_ERROR("SD_Init.PowON:%d\r\n", error);
		return error;
	}

	/* After power-on or CMD0, all cards?
	 * CMD lines are in input mode, waiting for start bit of the next command.
	 * The cards are initialized with a default relative card address
	 * (RCA=0x0000) and with a default driver stage register setting
	 * (lowest speed, highest driving current capability). */
	error = SdMmcIdentify(pSd);

	if (error) {
		TRACE_ERROR("SD_Init.Identify: %d\r\n", error);
		return error;
	}

	error = SdMmcEnum(pSd);

	if (error) {
		TRACE_ERROR("SD_Init.Enum: %d\r\n", error);
		return error;
	}

	/* In the case of a Standard Capacity SD Memory Card, this command sets the
	 * block length (in bytes) for all following block commands
	 * (read, write, lock).
	 * Default block length is fixed to 512 Bytes.
	 * Set length is valid for memory access commands only if partial block read
	 * operation are allowed in CSD.
	 * In the case of a High Capacity SD Memory Card, block length set by CMD16
	 * command does not affect the memory read and write commands. Always 512
	 * Bytes fixed block length is used. This command is effective for
	 * LOCK_UNLOCK command.
	 * In both cases, if block length is set larger than 512Bytes, the card sets
	 * the BLOCK_LEN_ERROR bit. */
	if (pSd->bCardType == CARD_SD) {
		error = Cmd16(pSd, SDMMC_BLOCK_SIZE);

		if (error) {
			TRACE_ERROR("SD_Init.Enum: %d\r\n", error);
			return error;
		}
	}

	pSd->wCurrBlockLen = SDMMC_BLOCK_SIZE;

	/* Reset status for R/W */
	pSd->bState = SDMMC_STATE_TRAN;


	/* If SD CSD v2.0 */
	if ((pSd->bCardType & CARD_TYPE_bmSDMMC) == CARD_TYPE_bmSD && SD_CSD_STRUCTURE(pSd->CSD) >= 1) {
		pSd->wBlockSize  = 512;
		pSd->dwNbBlocks  = SD_CSD_BLOCKNR_HC(pSd->CSD);
		pSd->dwTotalSize = 0xFFFFFFFF;
	}
	/* Normal SD/MMC card */
  else if (pSd->bCardType & CARD_TYPE_bmSDMMC) {
		pSd->wBlockSize  = 512; //SD_CSD_BLOCK_LEN(pSd->CSD);
		pSd->dwTotalSize = SD_CSD_TOTAL_SIZE(pSd->CSD);
		pSd->dwNbBlocks  = pSd->dwTotalSize / 512; //SD_CSD_BLOCKNR(pSd->CSD);
	}

	if (pSd->bCardType == CARD_UNKNOWN)
		return SDMMC_ERROR_NOT_INITIALIZED;

	/* Automatically select the max clock */
	clock = pSd->dwTranSpeed;
	_HwSetClock(pSd, &clock);
	TRACE_WARNING_WP("-I- Set SD clock to %uK\r\n", (unsigned int)clock / 1000);
	pSd->dwCurrSpeed = clock;
	return 0;
}

/**
 * De-initialize the driver. Invoked when SD card disconnected.
 * \param pSd  Pointer to a SD card driver instance.
 */
void SD_DeInit(sSdCard *pSd)
{
	if (pSd->pDrv) {
		_HwReset(pSd);
		_SdParamReset(pSd);
	}
}

/**
 * Return type of the card.
 * \param pSd Pointer to \ref sSdCard instance.
 * \sa sdmmc_cardtype
 */
uint8_t SD_GetCardType(sSdCard *pSd)
{
	assert(pSd != NULL);

	return pSd->bCardType;
}

/**
 * Return size of the SD/MMC card, in KB.
 * \param pSd Pointer to \ref sSdCard instance.
 */
uint32_t SD_GetTotalSizeKB(sSdCard *pSd)
{
	assert(pSd != NULL);

	if (pSd->dwTotalSize == 0xFFFFFFFF)
		return pSd->dwNbBlocks / 2;

	return pSd->dwTotalSize / 1024;
}

/**
 * Return reported block size of the SD/MMC card.
 * (SD/MMC access block size is always 512B for R/W).
 * \param pSd Pointer to \ref sSdCard instance.
 */
uint32_t SD_GetBlockSize(sSdCard *pSd)
{
	assert(pSd != NULL);

	return pSd->wBlockSize;
}

/**
 * Return reported number of blocks for the SD/MMC card.
 * \param pSd Pointer to \ref sSdCard instance.
 */
uint32_t SD_GetNumberBlocks(sSdCard *pSd)
{
	assert(pSd != NULL);

	return pSd->dwNbBlocks;
}




/**
 * Display SDIO card informations (CIS, tuple ...)
 * \param pSd Pointer to \ref sSdCard instance.
 */
void SDIO_DumpCardInformation(sSdCard *pSd)
{
	uint32_t tmp = 0, addrCIS = 0, addrManfID = 0, addrFunc0 = 0;
	uint8_t *p = (uint8_t *)&tmp;
	uint8_t buf[8];

	//printf("** trace : %d %x %X\r\n", DYN_TRACES, TRACE_LEVEL, dwTraceLevel);
	switch (pSd->bCardType) {
	case CARD_SDIO:
		TRACE_INFO("** SDIO ONLY card\r\n");
		break;

	case CARD_SDCOMBO: case CARD_SDHCCOMBO:
		TRACE_INFO("** SDIO Combo card\r\n");
		break;

	default:
		TRACE_INFO("** NO SDIO\r\n");
		return;
	}

	/* CCCR */
	TRACE_INFO("====== CCCR ======\r\n");
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_CCCR_REG, p, 1);
	TRACE_INFO(".SDIO       %02lX\r\n", (tmp & SDIO_SDIO) >> 4);
	TRACE_INFO(".CCCR       %02lX\r\n", (tmp & SDIO_CCCR) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_SD_REV_REG, p, 1);
	TRACE_INFO(".SD         %02lX\r\n", (tmp & SDIO_SD) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_IOE_REG, p, 1);
	TRACE_INFO(".IOE        %02lX\r\n", (tmp & SDIO_IOE) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_IOR_REG, p, 1);
	TRACE_INFO(".IOR        %02lX\r\n", (tmp & SDIO_IOR) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_IEN_REG, p, 1);
	TRACE_INFO(".IEN        %02lX\r\n", (tmp & SDIO_IEN) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_INT_REG, p, 1);
	TRACE_INFO(".INT        %u\r\n", (unsigned int)(tmp & SDIO_INT));
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_BUS_CTRL_REG, p, 1);
	TRACE_INFO(".CD         %lx\r\n", (tmp & SDIO_CD) >> 7);
	TRACE_INFO(".SCSI       %lx\r\n", (tmp & SDIO_SCSI) >> 6);
	TRACE_INFO(".ECSI       %lx\r\n", (tmp & SDIO_ECSI) >> 5);
	TRACE_INFO(".BUS_WIDTH  %lx\r\n", (tmp & SDIO_BUSWIDTH) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_CAP_REG, p, 1);
	TRACE_INFO(".4BLS       %lx\r\n", (tmp & SDIO_4BLS) >> 7);
	TRACE_INFO(".LSC        %lx\r\n", (tmp & SDIO_LSC) >> 6);
	TRACE_INFO(".E4MI       %lx\r\n", (tmp & SDIO_E4MI) >> 5);
	TRACE_INFO(".S4MI       %lx\r\n", (tmp & SDIO_S4MI) >> 4);
	TRACE_INFO(".SBS        %lx\r\n", (tmp & SDIO_SBS) >> 3);
	TRACE_INFO(".SRW        %lx\r\n", (tmp & SDIO_SRW) >> 2);
	TRACE_INFO(".SMB        %lx\r\n", (tmp & SDIO_SMB) >> 1);
	TRACE_INFO(".SDC        %lx\r\n", (tmp & SDIO_SDC) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_CIS_PTR_REG, p, 3);
	TRACE_INFO(".CIS_PTR    %06X\r\n", (unsigned int)tmp);
	addrCIS = tmp; tmp = 0;
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_BUS_SUSP_REG, p, 1);
	TRACE_INFO(".BR         %lx\r\n", (tmp & SDIO_BR) >> 1);
	TRACE_INFO(".BS         %lx\r\n", (tmp & SDIO_BS) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_FUN_SEL_REG, p, 1);
	TRACE_INFO(".DF         %lx\r\n", (tmp & SDIO_DF) >> 7);
	TRACE_INFO(".FS         %lx\r\n", (tmp & SDIO_FS) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_EXEC_REG, p, 1);
	TRACE_INFO(".EX         %lx\r\n", (tmp & SDIO_EX));
	TRACE_INFO(".EXM        %lx\r\n", (tmp & SDIO_EXM) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_READY_REG, p, 1);
	TRACE_INFO(".RF         %lx\r\n", (tmp & SDIO_RF));
	TRACE_INFO(".RFM        %lx\r\n", (tmp & SDIO_RFM) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_FN0_BLKSIZ_REG, p, 2);
	TRACE_INFO(".FN0_SIZE   %u(%04X)\r\n", (unsigned int)tmp, (unsigned int)tmp);
	tmp = 0;
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_POWER_REG, p, 1);
	TRACE_INFO(".EMPC       %lx\r\n", (tmp & SDIO_EMPC) >> 1);
	TRACE_INFO(".SMPC       %lx\r\n", (tmp & SDIO_SMPC) >> 0);
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_HS_REG, p, 1);
	TRACE_INFO(".EHS        %lx\r\n", (tmp & SDIO_EHS) >> 1);
	TRACE_INFO(".SHS        %lx\r\n", (tmp & SDIO_SHS) >> 0);
	/* Metaformat */
	SdioFindTuples(pSd, addrCIS, 128, &addrManfID, &addrFunc0);

	if (addrManfID != 0) {
		SDIO_ReadDirect(pSd, SDIO_CIA, addrManfID, buf, 6);
		TRACE_INFO("==== CISTPL_MANFID ====\r\n");
		TRACE_INFO("._MANF %04X\r\n", buf[2] + (buf[3] << 8));
		TRACE_INFO("._CARD %04X\r\n", buf[4] + (buf[5] << 8));
	}

	if (addrFunc0 != 0) {
		SDIO_ReadDirect(pSd, SDIO_CIA, addrFunc0, buf, 6);
		TRACE_INFO("== CISTPL_FUNCE Fun0 ==\r\n");
		TRACE_INFO("._FN0_BLK_SIZE   %d(0x%04X)\r\n",
				   buf[3] + (buf[4] << 8), buf[3] + (buf[4] << 8));
		TRACE_INFO("._MAX_TRAN_SPEED %02X\r\n", buf[5]);
	}
}


/**
 * Display the content of the CID register
 * \param pCID Pointer to CID data.
 */
void SD_DumpCID(void *pCID)
{
	TRACE_INFO("======= CID =======");
	//_DumpREG(pCID, 128/8);
	TRACE_INFO("===================\r\n");
	TRACE_INFO(" .MID Manufacturer ID             %02X\r\n",
			   (unsigned int)SD_CID_MID(pCID));

	TRACE_INFO(" .CBX Card/BGA (eMMC)             %X\r\n",
			   (unsigned int)eMMC_CID_CBX(pCID));

	TRACE_INFO(" .OID OEM/Application ID (SD)     %c%c\r\n",
			   (char)SD_CID_OID1(pCID),
			   (char)SD_CID_OID0(pCID));
	TRACE_INFO(" .OID OEM/Application ID (MMC)    %x\r\n",
			   (unsigned int)eMMC_CID_OID(pCID));

	TRACE_INFO(" .PNM Product name (SD)           %c%c%c%c%c\r\n",
			   (char)SD_CID_PNM4(pCID),
			   (char)SD_CID_PNM3(pCID),
			   (char)SD_CID_PNM2(pCID),
			   (char)SD_CID_PNM1(pCID),
			   (char)SD_CID_PNM0(pCID));
	TRACE_INFO(" .PNM Product name (MMC)          %c%c%c%c%c%c\r\n",
			   (char)MMC_CID_PNM5(pCID),
			   (char)MMC_CID_PNM4(pCID),
			   (char)MMC_CID_PNM3(pCID),
			   (char)MMC_CID_PNM2(pCID),
			   (char)MMC_CID_PNM1(pCID),
			   (char)MMC_CID_PNM0(pCID));

	TRACE_INFO(" .PRV Product revision (SD)       %x\r\n",
			   (unsigned int)SD_CID_PRV(pCID));
	TRACE_INFO(" .PRV Product revision (MMC)      %x\r\n",
			   (unsigned int)MMC_CID_PRV(pCID));

	TRACE_INFO(" .PSN Product serial number (SD)  %02X%02X%02X%02X\r\n",
			   (unsigned int)SD_CID_PSN3(pCID),
			   (unsigned int)SD_CID_PSN2(pCID),
			   (unsigned int)SD_CID_PSN1(pCID),
			   (unsigned int)SD_CID_PSN0(pCID));
	TRACE_INFO(" .PSN Product serial number (MMC) %02X%02X%02X%02X\r\n",
			   (unsigned int)MMC_CID_PSN3(pCID),
			   (unsigned int)MMC_CID_PSN2(pCID),
			   (unsigned int)MMC_CID_PSN1(pCID),
			   (unsigned int)MMC_CID_PSN0(pCID));

	TRACE_INFO(" .MDT Manufacturing date (SD)     %04d/%02d\r\n",
			   (uint16_t)(SD_CID_MDT_Y(pCID) + 2000),
			   (uint8_t)SD_CID_MDT_M(pCID));
	TRACE_INFO(" .MDT Manufacturing date (MMC)    %04d/%02d\r\n",
			   (uint16_t)(MMC_CID_MDT_Y(pCID) + 1997),
			   (uint8_t)SD_CID_MDT_M(pCID));

	TRACE_INFO(" .CRC checksum              %02X\r\n",
			   (unsigned int)SD_CID_CRC(pCID));
}

/**
 * Display the content of the CSD register
 * \param pSd Pointer to \ref sSdCard instance.
 */
void SD_DumpCSD(void *pCSD)
{
	TRACE_INFO("======== CSD ========");
	//_DumpREG(pCSD, 128/8);
	TRACE_INFO("===================\r\n");
	TRACE_INFO(" .CSD_STRUCTURE      0x%x\r\n",
			   (unsigned int)SD_CSD_STRUCTURE(pCSD));
	TRACE_INFO(" .SPEC_VERS (eMMC)   0x%x\r\n",
			   (unsigned int)MMC_CSD_SPEC_VERS(pCSD));
	TRACE_INFO(" .TAAC               0x%X\r\n",
			   (unsigned int)SD_CSD_TAAC(pCSD));
	TRACE_INFO(" .NSAC               0x%X\r\n",
			   (unsigned int)SD_CSD_NSAC(pCSD));
	TRACE_INFO(" .TRAN_SPEED         0x%X\r\n",
			   (unsigned int)SD_CSD_TRAN_SPEED(pCSD));
	TRACE_INFO(" .CCC                0x%X\r\n",
			   (unsigned int)SD_CSD_CCC(pCSD));
	TRACE_INFO(" .READ_BL_LEN        0x%X\r\n",
			   (unsigned int)SD_CSD_READ_BL_LEN(pCSD));
	TRACE_INFO(" .READ_BL_PARTIAL    0x%X\r\n",
			   (unsigned int)SD_CSD_READ_BL_PARTIAL(pCSD));
	TRACE_INFO(" .WRITE_BLK_MISALIGN 0x%X\r\n",
			   (unsigned int)SD_CSD_WRITE_BLK_MISALIGN(pCSD));
	TRACE_INFO(" .READ_BLK_MISALIGN  0x%X\r\n",
			   (unsigned int)SD_CSD_READ_BLK_MISALIGN(pCSD));
	TRACE_INFO(" .DSR_IMP            0x%X\r\n",
			   (unsigned int)SD_CSD_DSR_IMP(pCSD));
	TRACE_INFO(" .C_SIZE             0x%X\r\n",
			   (unsigned int)SD_CSD_C_SIZE(pCSD));
	TRACE_INFO(" .C_SIZE_HC          0x%X\r\n",
			   (unsigned int)SD2_CSD_C_SIZE(pCSD));
	TRACE_INFO(" .VDD_R_CURR_MIN     0x%X\r\n",
			   (unsigned int)SD_CSD_VDD_R_CURR_MIN(pCSD));
	TRACE_INFO(" .VDD_R_CURR_MAX     0x%X\r\n",
			   (unsigned int)SD_CSD_VDD_R_CURR_MAX(pCSD));
	TRACE_INFO(" .VDD_W_CURR_MIN     0x%X\r\n",
			   (unsigned int)SD_CSD_VDD_W_CURR_MIN(pCSD));
	TRACE_INFO(" .VDD_W_CURR_MAX     0x%X\r\n",
			   (unsigned int)SD_CSD_VDD_W_CURR_MAX(pCSD));
	TRACE_INFO(" .C_SIZE_MULT        0x%X\r\n",
			   (unsigned int)SD_CSD_C_SIZE_MULT(pCSD));
	TRACE_INFO(" .ERASE_BLK_EN       0x%X\r\n",
			   (unsigned int)SD_CSD_ERASE_BLK_EN(pCSD));
	TRACE_INFO(" .SECTOR_SIZE        0x%X\r\n",
			   (unsigned int)SD_CSD_SECTOR_SIZE(pCSD));
	TRACE_INFO(" .WP_GRP_SIZE        0x%X\r\n",
			   (unsigned int)SD_CSD_WP_GRP_SIZE(pCSD));
	TRACE_INFO(" .WP_GRP_ENABLE      0x%X\r\n",
			   (unsigned int)SD_CSD_WP_GRP_ENABLE(pCSD));
	TRACE_INFO(" .R2W_FACTOR         0x%X\r\n",
			   (unsigned int)SD_CSD_R2W_FACTOR(pCSD));
	TRACE_INFO(" .WRITE_BL_LEN       0x%X\r\n",
			   (unsigned int)SD_CSD_WRITE_BL_LEN(pCSD));
	TRACE_INFO(" .WRITE_BL_PARTIAL   0x%X\r\n",
			   (unsigned int)SD_CSD_WRITE_BL_PARTIAL(pCSD));
	TRACE_INFO(" .FILE_FORMAT_GRP    0x%X\r\n",
			   (unsigned int)SD_CSD_FILE_FORMAT_GRP(pCSD));
	TRACE_INFO(" .COPY               0x%X\r\n",
			   (unsigned int)SD_CSD_COPY(pCSD));
	TRACE_INFO(" .PERM_WRITE_PROTECT 0x%X\r\n",
			   (unsigned int)SD_CSD_PERM_WRITE_PROTECT(pCSD));
	TRACE_INFO(" .TMP_WRITE_PROTECT  0x%X\r\n",
			   (unsigned int)SD_CSD_TMP_WRITE_PROTECT(pCSD));
	TRACE_INFO(" .FILE_FORMAT        0x%X\r\n",
			   (unsigned int)SD_CSD_FILE_FORMAT(pCSD));
	TRACE_INFO(" .ECC (MMC)          0x%X\r\n",
			   (unsigned int)MMC_CSD_ECC(pCSD));
	TRACE_INFO(" .CRC                0x%X\r\n",
			   (unsigned int)SD_CSD_CRC(pCSD));
	TRACE_INFO(" .MULT               0x%X\r\n",
			   (unsigned int)SD_CSD_MULT(pCSD));
	TRACE_INFO(" .BLOCKNR            0x%X\r\n",
			   (unsigned int)SD_CSD_BLOCKNR(pCSD));
	TRACE_INFO(" .BLOCKNR_HC         0x%X\r\n",
			   (unsigned int)SD_CSD_BLOCKNR_HC(pCSD));
	TRACE_INFO(" .BLOCK_LEN          0x%X\r\n",
			   (unsigned int)SD_CSD_BLOCK_LEN(pCSD));
	TRACE_INFO(" -TOTAL_SIZE         0x%X\r\n",
			   (unsigned int)SD_CSD_TOTAL_SIZE(pCSD));
	TRACE_INFO(" -TOTAL_SIZE_HC      0x%X\r\n",
			   (unsigned int)SD_CSD_TOTAL_SIZE_HC(pCSD));
}

/**
 * Display the content of the EXT_CSD register
 * \param pExtCSD Pointer to extended CSD data.
 */
void SD_DumpExtCSD(void *pExtCSD)
{
	TRACE_INFO("======= EXT_CSD =======");
	TRACE_INFO_WP("\r\n");
	TRACE_INFO(" .S_CMD_SET            : 0x%X\r\n",
			   MMC_EXT_S_CMD_SET(pExtCSD));
	TRACE_INFO(" .BOOT_INFO            : 0x%X\r\n",
			   MMC_EXT_BOOT_INFO(pExtCSD));
	TRACE_INFO(" .BOOT_SIZE_MULTI      : 0x%X\r\n",
			   MMC_EXT_BOOT_SIZE_MULTI(pExtCSD));
	TRACE_INFO(" .ACC_SIZE             : 0x%X\r\n",
			   MMC_EXT_ACC_SIZE(pExtCSD));
	TRACE_INFO(" .HC_ERASE_GRP_SIZE    : 0x%X\r\n",
			   MMC_EXT_HC_ERASE_GRP_SIZE(pExtCSD));
	TRACE_INFO(" .ERASE_TIMEOUT_MULT   : 0x%X\r\n",
			   MMC_EXT_ERASE_TIMEOUT_MULT(pExtCSD));
	TRACE_INFO(" .REL_WR_SEC_C         : 0x%X\r\n",
			   MMC_EXT_REL_WR_SEC_C(pExtCSD));
	TRACE_INFO(" .HC_WP_GRP_SIZE       : 0x%X\r\n",
			   MMC_EXT_HC_WP_GRP_SIZE(pExtCSD));
	TRACE_INFO(" .S_C_VCC              : 0x%X\r\n",
			   MMC_EXT_S_C_VCC(pExtCSD));
	TRACE_INFO(" .S_C_VCCQ             : 0x%X\r\n",
			   MMC_EXT_S_C_VCCQ(pExtCSD));
	TRACE_INFO(" .S_A_TIMEOUT          : 0x%X\r\n",
			   MMC_EXT_S_A_TIMEOUT(pExtCSD));
	TRACE_INFO(" .SEC_COUNT            : 0x%X\r\n",
			   MMC_EXT_SEC_COUNT(pExtCSD));
	TRACE_INFO(" .MIN_PERF_W_8_52      : 0x%X\r\n",
			   MMC_EXT_MIN_PERF_W_8_52(pExtCSD));
	TRACE_INFO(" .MIN_PERF_R_8_52      : 0x%X\r\n",
			   MMC_EXT_MIN_PERF_R_8_52(pExtCSD));
	TRACE_INFO(" .MIN_PERF_W_8_26_4_52 : 0x%X\r\n",
			   MMC_EXT_MIN_PERF_W_8_26_4_52(pExtCSD));
	TRACE_INFO(" .MIN_PERF_R_8_26_4_52 : 0x%X\r\n",
			   MMC_EXT_MIN_PERF_R_8_26_4_52(pExtCSD));
	TRACE_INFO(" .MIN_PERF_W_4_26      : 0x%X\r\n",
			   MMC_EXT_MIN_PERF_W_4_26(pExtCSD));
	TRACE_INFO(" .MIN_PERF_R_4_26      : 0x%X\r\n",
			   MMC_EXT_MIN_PERF_R_4_26(pExtCSD));
	TRACE_INFO(" .PWR_CL_26_360        : 0x%X\r\n",
			   MMC_EXT_PWR_CL_26_360(pExtCSD));
	TRACE_INFO(" .PWR_CL_52_360        : 0x%X\r\n",
			   MMC_EXT_PWR_CL_52_360(pExtCSD));
	TRACE_INFO(" .PWR_CL_26_195        : 0x%X\r\n",
			   MMC_EXT_PWR_CL_26_195(pExtCSD));
	TRACE_INFO(" .PWR_CL_52_195        : 0x%X\r\n",
			   MMC_EXT_PWR_CL_52_195(pExtCSD));
	TRACE_INFO(" .CARD_TYPE            : 0x%X\r\n",
			   MMC_EXT_CARD_TYPE(pExtCSD));
	TRACE_INFO(" .CSD_STRUCTURE        : 0x%X\r\n",
			   MMC_EXT_CSD_STRUCTURE(pExtCSD));
	TRACE_INFO(" .EXT_CSD_REV          : 0x%X\r\n",
			   MMC_EXT_EXT_CSD_REV(pExtCSD));
	TRACE_INFO(" .CMD_SET              : 0x%X\r\n",
			   MMC_EXT_CMD_SET(pExtCSD));
	TRACE_INFO(" .CMD_SET_REV          : 0x%X\r\n",
			   MMC_EXT_CMD_SET_REV(pExtCSD));
	TRACE_INFO(" .POWER_CLASS          : 0x%X\r\n",
			   MMC_EXT_POWER_CLASS(pExtCSD));
	TRACE_INFO(" .HS_TIMING            : 0x%X\r\n",
			   MMC_EXT_HS_TIMING(pExtCSD));
	TRACE_INFO(" .BUS_WIDTH            : 0x%X\r\n",
			   MMC_EXT_BUS_WIDTH(pExtCSD));
	TRACE_INFO(" .ERASED_MEM_CONT      : 0x%X\r\n",
			   MMC_EXT_ERASED_MEM_CONT(pExtCSD));
	TRACE_INFO(" .BOOT_CONFIG          : 0x%X\r\n",
			   MMC_EXT_BOOT_CONFIG(pExtCSD));
	TRACE_INFO(" .BOOT_BUS_WIDTH       : 0x%X\r\n",
			   MMC_EXT_BOOT_BUS_WIDTH(pExtCSD));
	TRACE_INFO(" .ERASE_GROUP_DEF      : 0x%X\r\n",
			   MMC_EXT_ERASE_GROUP_DEF(pExtCSD));
}

/**
 * Display the content of the SCR register
 * \param pSCR  Pointer to SCR data.
 */
void SD_DumpSCR(void *pSCR)
{
	TRACE_INFO("========== SCR ==========");
	TRACE_INFO_WP("\r\n");

	TRACE_INFO(" .SCR_STRUCTURE         :0x%X\r\n",
			   (unsigned int)SD_SCR_STRUCTURE(pSCR));
	TRACE_INFO(" .SD_SPEC               :0x%X\r\n",
			   (unsigned int)SD_SCR_SD_SPEC(pSCR));
	TRACE_INFO(" .DATA_STAT_AFTER_ERASE :0x%X\r\n",
			   (unsigned int)SD_SCR_DATA_STAT_AFTER_ERASE(pSCR));
	TRACE_INFO(" .SD_SECURITY           :0x%X\r\n",
			   (unsigned int)SD_SCR_SD_SECURITY(pSCR));
	TRACE_INFO(" .SD_BUS_WIDTHS         :0x%X\r\n",
			   (unsigned int)SD_SCR_SD_BUS_WIDTHS(pSCR));
}

/**
 * Display the content of the SD Status
 * \param pSdST  Pointer to SD card status data.
 */
void SD_DumpSdStatus(void *pSdST)
{
	/*The swap operation only can be done once*/
	if (0 == ssr_is_dump) {
		uint32_t  i;
		uint32_t *pTemp = (uint32_t *)pSdST;

		for (i = 0; i < 512 / 8 / 4; i++)
			pTemp[i] = __REV(pTemp[i]);

		ssr_is_dump = 1;
	}

	TRACE_INFO("=========== STAT ============");
	TRACE_INFO_WP("\r\n");
	TRACE_INFO(" .DAT_BUS_WIDTH          :0x%X\r\n",
			   (unsigned int)SD_ST_DAT_BUS_WIDTH(pSdST));
	TRACE_INFO(" .SECURED_MODE           :0x%X\r\n",
			   (unsigned int)SD_ST_SECURED_MODE(pSdST));
	TRACE_INFO(" .SD_CARD_TYPE           :0x%X\r\n",
			   (unsigned int)SD_ST_CARD_TYPE(pSdST));
	TRACE_INFO(" .SIZE_OF_PROTECTED_AREA :0x%X\r\n",
			   (unsigned int)SD_ST_SIZE_OF_PROTECTED_AREA(pSdST));
	TRACE_INFO(" .SPEED_CLASS            :0x%X\r\n",
			   (unsigned int)SD_ST_SPEED_CLASS(pSdST));
	TRACE_INFO(" .PERFORMANCE_MOVE       :0x%X\r\n",
			   (unsigned int)SD_ST_PERFORMANCE_MOVE(pSdST));
	TRACE_INFO(" .AU_SIZE                :0x%X\r\n",
			   (unsigned int)SD_ST_AU_SIZE(pSdST));
	TRACE_INFO(" .ERASE_SIZE             :0x%X\r\n",
			   (unsigned int)SD_ST_ERASE_SIZE(pSdST));
	TRACE_INFO(" .ERASE_TIMEOUT          :0x%X\r\n",
			   (unsigned int)SD_ST_ERASE_TIMEOUT(pSdST));
	TRACE_INFO(" .ERASE_OFFSET           :0x%X\r\n",
			   (unsigned int)SD_ST_ERASE_OFFSET(pSdST));

}
/**@}*/
