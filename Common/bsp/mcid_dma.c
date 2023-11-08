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

/** \file
 *
 *  Implement for SD/MMC low level commands.
 *
 *  \sa \ref hsmci_module, \ref sdmmc_module
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "sdmmc.h"

#include <xdmad.h>
#include <mcid.h>
#include <pmc.h>
#include <assert.h>
#include <sdmmc_trace.h>
#include <hsmci.h>

/*----------------------------------------------------------------------------
 *         Local constants
 *----------------------------------------------------------------------------*/
/** \addtorgoup mcid_defines
 *      @{*/

/** Enable MCI */
#define MCI_ENABLE(pMciHw)     HSMCI_Enable(pMciHw)
/** Disable MCI */
#define MCI_DISABLE(pMciHw)    HSMCI_Disable(pMciHw)
/** Reset MCI */
#define MCI_RESET(pMciHw)      HSMCI_Reset(pMciHw, 0)

/** Return halfword(16-bit) count from byte count */
#define toHWCOUNT(byteCnt) (((byteCnt)&0x1) ? (((byteCnt)/2)+1) : ((byteCnt)/2))
/** Return word(32-bit) count from byte count */
#define toWCOUNT(byteCnt)  (((byteCnt)&0x3) ? (((byteCnt)/4)+1) : ((byteCnt)/4))


/** Bit mask for status register errors. */
#define STATUS_ERRORS ((uint32_t)(HSMCI_SR_UNRE  \
								  | HSMCI_SR_OVRE \
								  | HSMCI_SR_ACKRCVE \
								  | HSMCI_SR_CSTOE \
								  | HSMCI_SR_DTOE \
								  | HSMCI_SR_DCRCE \
								  | HSMCI_SR_RTOE \
								  | HSMCI_SR_RENDE \
								  | HSMCI_SR_RCRCE \
								  | HSMCI_SR_RDIRE \
								  | HSMCI_SR_RINDE))

/** Bit mask for response errors */
#define STATUS_ERRORS_RESP ((uint32_t)(HSMCI_SR_CSTOE \
									   | HSMCI_SR_RTOE \
									   | HSMCI_SR_RENDE \
									   | HSMCI_SR_RCRCE \
									   | HSMCI_SR_RDIRE \
									   | HSMCI_SR_RINDE))

/** Bit mask for data errors */
#define STATUS_ERRORS_DATA ((uint32_t)(HSMCI_SR_UNRE \
									   | HSMCI_SR_OVRE \
									   | HSMCI_SR_DTOE \
									   | HSMCI_SR_DCRCE))


/** Moved from xdmac.h -- value does not make sense? */
#define XDMAC_MAX_BT_SIZE               0xFFFF
/** Max DMA size in a single transfer */
#define MAX_DMA_SIZE                (XDMAC_MAX_BT_SIZE & 0xFFFFFF00)


/** SD/MMC memory Single block */
#define _CMDR_SDMEM_SINGLE  (HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRTYP_SINGLE)
/** SD/MMC memory Multi block */
#define _CMDR_SDMEM_MULTI   (HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRTYP_MULTIPLE)
/** SDIO byte transfer */
#define _CMDR_SDIO_BYTE     (HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRTYP_BYTE)
/** SDIO block transfer */
#define _CMDR_SDIO_BLOCK    (HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRTYP_BLOCK)

/**     @}*/
/*---------------------------------------------------------------------------
 *         Local types
 *---------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *         Local variable
 *----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/** \addtogroup mcid_functions
 *@{
 */

//#ifdef USE_DEPRECATED
static void hsmci_callback(uint32_t channel, sMcid *pMcid)
{
  /*
    Here's the deal: DCache implementation sucks in SoftPack.
    So we're moving all DMA buffers from SRAM into TCM which is uncached...
  */
  
  channel = channel;
//	sSdmmcCommand *pCmd = pMcid->pCmd;
//	SCB_InvalidateDCache_by_Addr((uint32_t*)&pCmd->pData[0], pCmd->wBlockSize * pCmd->wNbBlocks);
  
}
//#endif

/**
 * HSMCI DMA R/W prepare
 */
static uint32_t _MciDMAPrepare(sMcid *pMcid, uint8_t bRd)
{
	sXdmad *pXdmad = pMcid->pXdmad;
  
//  pMcid->dwDmaCh = XDMAD_AllocateChannel(pXdmad);
//  XDMAD_SetCallback(pXdmad, pMcid->dwDmaCh, NULL, NULL);

  XDMAD_SetCallback(pXdmad, pMcid->dwDmaCh, (XdmadTransferCallback)hsmci_callback, pMcid);

	if (pMcid->dwDmaCh == XDMAD_ALLOC_FAILED)
		return SDMMC_ERROR_BUSY;

	XDMAD_PrepareChannel(pXdmad, pMcid->dwDmaCh);
	return SDMMC_SUCCESS;
}

/**
 * HSMCI DMA R/W
 * \return 1 if DMA started.
 */

/* Linked lists for multi transfer buffer chaining structure instance. */
COMPILER_SECTION(".ram_nocache") COMPILER_ALIGNED(32) static LinkedListDescriporView1 dmaLinkList[256];

static uint32_t _MciDMA(sMcid *pMcid, uint32_t bFByte, uint8_t bRd)
{
	Hsmci *pHw = pMcid->pMciHw;
	sXdmad *pXdmad = pMcid->pXdmad;
	sSdmmcCommand *pCmd = pMcid->pCmd;
	sXdmadCfg xdmadRxCfg, xdmadTxCfg;
	uint32_t xdmaCndc, xdmaInt;
	uint8_t i;
	uint32_t totalSize = pCmd->wNbBlocks * pCmd->wBlockSize;
	uint32_t maxXSize;
	uint32_t memAddress;
	uint8_t  bMByte;

	if (pMcid->dwXfrNdx >= totalSize) return 0;

	/* Prepare DMA transfer */
	if (pCmd->wBlockSize != 1) {
		pMcid->dwXSize = totalSize - pMcid->dwXfrNdx;

		if (bRd) {
			for (i = 0; i < pCmd->wNbBlocks; i++) {
				dmaLinkList[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1
										 | ((i == pCmd->wNbBlocks - 1) ? 0 : XDMA_UBC_NDE_FETCH_EN)
										 | XDMA_UBC_NDEN_UPDATED
										 | pCmd->wBlockSize / 4;
				dmaLinkList[i].mbr_sa  = (uint32_t) & (pHw->HSMCI_FIFO[i]);
				dmaLinkList[i].mbr_da = (uint32_t)&pCmd->pData[i * pCmd->wBlockSize];

				if (i == pCmd->wNbBlocks - 1)
					dmaLinkList[i].mbr_nda = 0;
				else
					dmaLinkList[i].mbr_nda = (uint32_t)&dmaLinkList[ i + 1 ];
          //SCB_CleanDCache_by_Addr((uint32_t *)&dmaLinkList[i],sizeof(LinkedListDescriporView1));
			}

			xdmadRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
								 | XDMAC_CC_MBSIZE_SINGLE
								 | XDMAC_CC_DSYNC_PER2MEM
								 | XDMAC_CC_CSIZE_CHK_1
								 | XDMAC_CC_DWIDTH_WORD
								 | XDMAC_CC_SIF_AHB_IF1
								 | XDMAC_CC_DIF_AHB_IF0
								 | XDMAC_CC_SAM_FIXED_AM
								 | XDMAC_CC_DAM_INCREMENTED_AM
								 | XDMAC_CC_PERID(DMA_HSMCI_RX);
			xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1
					   | XDMAC_CNDC_NDE_DSCR_FETCH_EN
					   | XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED
					   | XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;


			if (XDMAD_ConfigureTransfer(pXdmad, pMcid->dwDmaCh,
										 &xdmadRxCfg, xdmaCndc, (uint32_t)&dmaLinkList[0],
										 XDMAC_CIE_LIE /*0*/))
				return 0;

			//SCB_CleanDCache_by_Addr((uint32_t *)dmaLinkList, sizeof(dmaLinkList));

			if (XDMAD_StartTransfer(pXdmad, pMcid->dwDmaCh))
				return 0;

			//Write
		} else {
			for (i = 0; i < pCmd->wNbBlocks; i++) {
				dmaLinkList[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1
										 | ((i == pCmd->wNbBlocks - 1) ? 0 : XDMA_UBC_NDE_FETCH_EN)
										 | XDMA_UBC_NDEN_UPDATED
										 | pCmd->wBlockSize / 4;
				dmaLinkList[i].mbr_sa = (uint32_t)&pCmd->pData[i * pCmd->wBlockSize];
				dmaLinkList[i].mbr_da = (uint32_t) & (pHw->HSMCI_FIFO[i]);

				// cache maintenance
        //SCB_CleanDCache_by_Addr((uint32_t *)&pCmd->pData[i * pCmd->wBlockSize],pCmd->wBlockSize);
				if (i == pCmd->wNbBlocks - 1) dmaLinkList[i].mbr_nda = 0;
				else dmaLinkList[i].mbr_nda = (uint32_t)&dmaLinkList[ i + 1 ];
        //SCB_CleanDCache_by_Addr((uint32_t *)&dmaLinkList[i],sizeof(LinkedListDescriporView1));

			}

			xdmadTxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
								 | XDMAC_CC_MBSIZE_SINGLE
								 | XDMAC_CC_DSYNC_MEM2PER
								 | XDMAC_CC_CSIZE_CHK_1
								 | XDMAC_CC_DWIDTH_WORD
								 | XDMAC_CC_SIF_AHB_IF0
								 | XDMAC_CC_DIF_AHB_IF1
								 | XDMAC_CC_SAM_INCREMENTED_AM
								 | XDMAC_CC_DAM_FIXED_AM
								 | XDMAC_CC_PERID(DMA_HSMCI_TX);
			xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1
					   | XDMAC_CNDC_NDE_DSCR_FETCH_EN
					   | XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED
					   | XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;

			if (XDMAD_ConfigureTransfer(pXdmad, pMcid->dwDmaCh, &xdmadTxCfg, xdmaCndc, (uint32_t)&dmaLinkList[0], /*XDMAC_CIE_LIE*/0))
				return 0;

			//SCB_CleanDCache_by_Addr((uint32_t *)&pCmd->pData[0], pCmd->wBlockSize * pCmd->wNbBlocks);
			//SCB_CleanDCache_by_Addr((uint32_t *)dmaLinkList, sizeof(dmaLinkList));

			if (XDMAD_StartTransfer(pXdmad, pMcid->dwDmaCh))
				return 0;
		}
	} else {
		/* Memory address and alignment */
		memAddress = (uint32_t)&pCmd->pData[pMcid->dwXfrNdx];
		bMByte = bFByte ? 1 : (((memAddress & 0x3) || (totalSize & 0x3)));

		/* P to M: Max size is P size */
		if (bRd)
			maxXSize = bFByte ? MAX_DMA_SIZE : (MAX_DMA_SIZE * 4);
		else {
			/* M to P: Max size is M size */
			maxXSize = bMByte ? MAX_DMA_SIZE : (MAX_DMA_SIZE * 4);
		}

		/* Update index */
		pMcid->dwXSize = totalSize - pMcid->dwXfrNdx;

		if (pMcid->dwXSize > maxXSize)
			pMcid->dwXSize = maxXSize;

		/* Prepare DMA transfer */
		if (bRd) {
			xdmadRxCfg.mbr_ubc = bFByte ? pMcid->dwXSize : toWCOUNT(pMcid->dwXSize);
			xdmadRxCfg.mbr_sa = (uint32_t) & (pHw->HSMCI_RDR);
			xdmadRxCfg.mbr_da = (uint32_t)memAddress;
			xdmadRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
								 XDMAC_CC_MEMSET_NORMAL_MODE |
								 XDMAC_CC_DSYNC_PER2MEM |
								 XDMAC_CC_CSIZE_CHK_1 |
								 (bFByte ? XDMAC_CC_DWIDTH_BYTE : XDMAC_CC_DWIDTH_WORD) |
								 XDMAC_CC_SIF_AHB_IF1 |
								 XDMAC_CC_DIF_AHB_IF1 |
								 XDMAC_CC_SAM_FIXED_AM |
								 XDMAC_CC_DAM_INCREMENTED_AM;
			xdmadRxCfg.mbr_bc = 0;
			xdmaInt =  (XDMAC_CIE_BIE   |
						XDMAC_CIE_DIE   |
						XDMAC_CIE_FIE   |
						XDMAC_CIE_RBIE  |
						XDMAC_CIE_WBIE  |
						XDMAC_CIE_ROIE);

			XDMAD_ConfigureTransfer(pXdmad, pMcid->dwDmaCh, &xdmadRxCfg, 0, 0, xdmaInt/*0*/);
		}
    else
    {
			//SCB_CleanDCache_by_Addr((uint32_t *)memAddress, pMcid->dwXSize);
			xdmadTxCfg.mbr_ubc = bFByte ? pMcid->dwXSize : toWCOUNT(pMcid->dwXSize);
			xdmadTxCfg.mbr_sa = (uint32_t)memAddress;
			xdmadTxCfg.mbr_da = (uint32_t) & (pHw->HSMCI_TDR);
			xdmadTxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
								 XDMAC_CC_MEMSET_NORMAL_MODE |
								 XDMAC_CC_DSYNC_MEM2PER |
								 XDMAC_CC_CSIZE_CHK_1 |
								 (bFByte ? XDMAC_CC_DWIDTH_BYTE : XDMAC_CC_DWIDTH_WORD) |
								 XDMAC_CC_SIF_AHB_IF1 |
								 XDMAC_CC_DIF_AHB_IF1 |
								 XDMAC_CC_SAM_INCREMENTED_AM |
								 XDMAC_CC_DAM_FIXED_AM;
			xdmadTxCfg.mbr_bc = 0;

			xdmaInt =  (XDMAC_CIE_BIE   |
						XDMAC_CIE_DIE   |
						XDMAC_CIE_FIE   |
						XDMAC_CIE_RBIE  |
						XDMAC_CIE_WBIE  |
						XDMAC_CIE_ROIE);
						
			XDMAD_ConfigureTransfer(pXdmad, pMcid->dwDmaCh, &xdmadTxCfg, 0, 0, /*xdmaInt*/ 0);
		}

		XDMAD_StartTransfer(pXdmad, pMcid->dwDmaCh);
	}

	return 1;
}

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * Reset MCI HW interface and disable it.
 * \param keepSettings Keep old register settings, including
 *                     _MR, _SDCR, _DTOR, _CSTOR, _DMA and _CFG.
 */
static void MCI_Reset(sMcid *pMci, uint8_t keepSettings)
{
	Hsmci *pMciHw = pMci->pMciHw;

	assert(pMci);
	assert(pMci->pMciHw);

	HSMCI_Reset(pMciHw, keepSettings);
}

/**
 * Configure the  MCI CLKDIV in the MCI_MR register. The max. for MCI clock is
 * MCK/2 and corresponds to CLKDIV = 0
 * \param pMci  Pointer to the low level MCI driver.
 * \param mciSpeed  MCI clock speed in Hz, 0 will not change current speed.
 * \param mck       MCK to generate MCI Clock, in Hz
 * \return The actual speed used, 0 for fail.
 */
static uint32_t MCI_SetSpeed(sMcid *pMci, uint32_t mciSpeed, uint32_t mck)
{
	Hsmci *pMciHw = pMci->pMciHw;
	uint32_t clkdiv;
	assert(pMci);
	assert(pMciHw);

	if ((mck % mciSpeed) == 0)
		clkdiv = mck / mciSpeed;
	else
		clkdiv = ((mck + mciSpeed) / mciSpeed);

	mciSpeed = mck / clkdiv;

	/* Modify MR */
	HSMCI_DivCtrl(pMciHw, clkdiv, 0x7);
	return (mciSpeed);
}

/**
*/
static void _FinishCmd(sMcid *pMcid, uint8_t bStatus)
{
	sSdmmcCommand *pCmd = pMcid->pCmd;
  /* Release DMA channel (if used) */
  /*
  uint32_t memAddress;
  sXdmad *pXdmad = pMcid->pXdmad;
	if (pMcid->dwDmaCh != XDMAD_ALLOC_FAILED) {
		if (XDMAD_FreeChannel(pXdmad, pMcid->dwDmaCh)) {
			TRACE_ERROR(" Can't free channel \r\n");
			TRACE_DEBUG(" Channel is in %d state\r\n", pXdmad->XdmaChannels[pMcid->dwDmaCh].state);
		}
  	pMcid->dwDmaCh = XDMAD_ALLOC_FAILED;
	}
  */

	/* Release command */
	pMcid->pCmd   = NULL;
	pMcid->bState = MCID_LOCKED;
	pCmd->bStatus = bStatus;

	/* Invoke callback */
	if (pCmd->fCallback)
		(pCmd->fCallback)(pCmd->bStatus, pCmd->pArg);
}

/*---------------------------------------------------------------------------
 *      Exported functions
 *---------------------------------------------------------------------------*/

/**
 * Select MCI slot.
 */
void MCID_SetSlot(Hsmci *pMci, uint8_t slot)

{
	HSMCI_SetSlot(pMci, slot);
}

/**
 * Initialize MCI driver.
 */
void MCID_Init(sMcid *pMcid,
			   Hsmci *pMci, uint8_t bID, uint32_t dwMck,
			   sXdmad *pXdmad,
			   uint8_t bPolling)
{
	assert(pMcid);
	assert(pMci);

	/* Initialize driver struct */
	pMcid->pMciHw    = pMci;
	pMcid->pCmd      = NULL;

	pMcid->pXdmad         = pXdmad;
	pMcid->dwDmaCh       = XDMAD_ALLOC_FAILED;
	pMcid->dwXfrNdx      = 0;

	pMcid->dwMck     = dwMck;

	pMcid->bID       = bID;
	pMcid->bPolling  = bPolling;
	pMcid->bState    = MCID_IDLE;

  pMcid->dwDmaCh = XDMAD_AllocateChannel(pXdmad);
  XDMAD_SetCallback(pXdmad, pMcid->dwDmaCh, NULL, NULL);

	PMC_EnablePeripheral(bID);

	MCI_RESET(pMci);
	MCI_DISABLE (pMci);
	HSMCI_DisableIt(pMci, 0xFFFFFFFF);
	HSMCI_ConfigureDataTO(pMci, HSMCI_DTOR_DTOCYC(0xFF)
						   | HSMCI_DTOR_DTOMUL_1048576);
	HSMCI_ConfigureCompletionTO(pMci , HSMCI_CSTOR_CSTOCYC(0xFF)
								 | HSMCI_CSTOR_CSTOMUL_1048576);
	/* Set the Mode Register: 400KHz */
	MCI_SetSpeed(pMcid,MCI_INITIAL_SPEED,dwMck);

	HSMCI_Enable(pMci);
	HSMCI_Configure(pMci, HSMCI_CFG_FIFOMODE | HSMCI_CFG_FERRCTRL);
	/* Enable DMA */
    HSMCI_EnableDma(pMci, 1);

}

/**
 * Lock the MCI driver for slot N access
 */
uint32_t MCID_Lock(sMcid *pMcid, uint8_t bSlot)
{
	Hsmci *pHw = pMcid->pMciHw;
	uint32_t sdcr;

	assert(pMcid);
	assert(pMcid->pMciHw);

	if (bSlot > 0)
		return SDMMC_ERROR_PARAM;

	if (pMcid->bState >= MCID_LOCKED)
		return SDMMC_ERROR_LOCKED;

	pMcid->bState = MCID_LOCKED;
	sdcr = pHw->HSMCI_SDCR & ~(uint32_t)HSMCI_SDCR_SDCSEL_Msk;
	pHw->HSMCI_SDCR = sdcr | (bSlot << HSMCI_SDCR_SDCSEL_Pos);
	return SDMMC_OK;
}

/**
 * Release the driver.
 */
/*
uint32_t MCID_Release(sMcid *pMcid)
{
	assert(pMcid);

	if (pMcid->bState >= MCID_CMD)
		return SDMMC_ERROR_BUSY;

	pMcid->bState = MCID_IDLE;
	return SDMMC_OK;
}
*/

/**
 * SD/MMC command.
 */
uint32_t MCID_SendCmd(sMcid *pMcid, void *pCommand)
{
	Hsmci *pHw = pMcid->pMciHw;
	sSdmmcCommand *pCmd = pCommand;
	uint32_t mr, ier;
	uint32_t cmdr;

	assert(pMcid);
	assert(pMcid->pMciHw);
	assert(pCmd);

	//printf("cmd = %d \r\n",pCmd->bCmd);
	if (!MCID_IsCmdCompleted(pMcid))
		return SDMMC_ERROR_BUSY;
    
  Board_SetLed(LED_SD, LED_TRI_G, LED_PRIO_NONE);

	pMcid->bState = MCID_CMD;
	pMcid->pCmd   = pCmd;

	MCI_DISABLE(pHw);
	mr = HSMCI_GetMode(pHw) & (~(uint32_t)(HSMCI_MR_WRPROOF |
										   HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE));

	/* Special: PowerON Init */
	if (pCmd->cmdOp.wVal == SDMMC_CMD_POWERONINIT) {
		HSMCI_ConfigureMode(pHw, mr);
		ier = HSMCI_IER_XFRDONE;
	} else if (pCmd->cmdOp.bmBits.xfrData == SDMMC_CMD_STOPXFR) {
		/* Normal command: idle the bus */
		HSMCI_ConfigureMode(pHw, mr);
		ier = HSMCI_IER_XFRDONE | STATUS_ERRORS_RESP;
	}
	/* No data transfer */
	else if ((pCmd->cmdOp.wVal & SDMMC_CMD_CNODATA(0xF)) == SDMMC_CMD_CNODATA(0)) {
		ier = HSMCI_IER_XFRDONE | STATUS_ERRORS_RESP;

		/* R3 response, no CRC */
		if (pCmd->cmdOp.bmBits.respType == 3)
			ier &= ~(uint32_t)HSMCI_IER_RCRCE;
	} else if (pCmd->wNbBlocks == 0 || pCmd->pData == 0) {
		/* Data command but no following */
		HSMCI_ConfigureMode(pHw, mr | HSMCI_MR_WRPROOF
							| HSMCI_MR_RDPROOF);
		HSMCI_ConfigureTransfer(pHw, pCmd->wBlockSize, pCmd->wNbBlocks);
		ier = HSMCI_IER_CMDRDY | STATUS_ERRORS_RESP;
	} else {
		/* Command with data */
		/* Setup block size */
		if (pCmd->cmdOp.bmBits.sendCmd)
			HSMCI_ConfigureTransfer(pHw, pCmd->wBlockSize, pCmd->wNbBlocks);

		/* Block size is 0, force byte */
		if (pCmd->wBlockSize == 0)
			pCmd->wBlockSize = 1;

		/* Force byte transfer */
		if (pCmd->wBlockSize & 0x3)
			mr |= HSMCI_MR_FBYTE;

		/* Set block size & MR */
		HSMCI_ConfigureMode(pHw, mr | HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF);

		/* DMA write */
		uint8_t byteMode = pCmd->cmdOp.bmBits.xfrData == SDMMC_CMD_TX ? 0 : 1;

#if 1
    if (_MciDMAPrepare(pMcid, byteMode))
    {
      _FinishCmd(pMcid, SDMMC_ERROR_BUSY);
      return SDMMC_ERROR_BUSY;
    }
#endif

    _MciDMA(pMcid, (mr & HSMCI_MR_FBYTE), byteMode);

    ier = HSMCI_IER_XFRDONE | STATUS_ERRORS_DATA;
#if 1
    if (pCmd->wNbBlocks > 1)
    {
      //ier |= HSMCI_IER_FIFOEMPTY;
    }
#endif   
  }

	MCI_ENABLE(pHw);

	if (pCmd->cmdOp.wVal & (SDMMC_CMD_bmPOWERON | SDMMC_CMD_bmCOMMAND)) {
		cmdr = pCmd->bCmd;

		if (pCmd->cmdOp.bmBits.powerON)
			cmdr |= (HSMCI_CMDR_OPDCMD | HSMCI_CMDR_SPCMD_INIT);

		if (pCmd->cmdOp.bmBits.odON)
			cmdr |= HSMCI_CMDR_OPDCMD;

		if (pCmd->cmdOp.bmBits.sendCmd)
			cmdr |= HSMCI_CMDR_MAXLAT;

		switch (pCmd->cmdOp.bmBits.xfrData) {
		case SDMMC_CMD_TX:
			if (pCmd->cmdOp.bmBits.ioCmd) {
				cmdr |= (pCmd->wBlockSize == 1) ?
						_CMDR_SDIO_BYTE :
						_CMDR_SDIO_BLOCK;
			} else {
				cmdr |= (pCmd->wNbBlocks == 1) ?
						_CMDR_SDMEM_SINGLE :
						_CMDR_SDMEM_MULTI;
			}

			break;

		case SDMMC_CMD_RX:
			if (pCmd->cmdOp.bmBits.ioCmd) {
				cmdr |= HSMCI_CMDR_TRDIR_READ
						| ((pCmd->wBlockSize == 1) ?
						   _CMDR_SDIO_BYTE :
						   _CMDR_SDIO_BLOCK);
			} else {
				cmdr |= HSMCI_CMDR_TRDIR_READ
						| ((pCmd->wNbBlocks == 1) ?
						   _CMDR_SDMEM_SINGLE :
						   _CMDR_SDMEM_MULTI);
			}

			break;

		case SDMMC_CMD_STOPXFR:
			cmdr |= HSMCI_CMDR_TRCMD_STOP_DATA;
			break;
		}

		switch (pCmd->cmdOp.bmBits.respType) {
		case 3: case 4:
			/* ignore CRC error */
			ier &= ~(uint32_t)HSMCI_IER_RCRCE;

		case 1: case 5: case 6: case 7:
			cmdr |= HSMCI_CMDR_RSPTYP_48_BIT;
			break;

		case 2:
			cmdr |= HSMCI_CMDR_RSPTYP_136_BIT;
			break;

		/* No response, ignore RTOE */
		default:
			ier &= ~(uint32_t)HSMCI_IER_RTOE;
		}

		pHw->HSMCI_ARGR = pCmd->dwArg;
		pHw->HSMCI_CMDR = cmdr;
	}

	/* Ignore CRC error for R3 & R4 */
	if (pCmd->cmdOp.bmBits.xfrData == SDMMC_CMD_STOPXFR)
		ier &= ~STATUS_ERRORS_DATA;

	/* Enable status flags */
	HSMCI_EnableIt(pHw, ier);
	return SDMMC_OK;
}

static uint32_t dwMsk;
/**
 * Process pending events on the given MCI driver.
 */

COMPILER_SECTION(".code_TCM")
void MCID_Handler(sMcid *pMcid)
{
	Hsmci *pHw = pMcid->pMciHw;
	sSdmmcCommand *pCmd = pMcid->pCmd;
	//uint32_t dwSr, dwMsk, dwMaskedSr;
	uint32_t dwSr, dwMaskedSr;
	assert(pMcid);
	assert(pMcid->pMciHw);

	/* Do nothing if no pending command */
	if (pCmd == NULL) {
		if (pMcid->bState >= MCID_CMD)
			pMcid->bState = MCID_LOCKED;

		return;
	}

	/* Read status */
	dwSr  = HSMCI_GetStatus(pHw);
	dwMsk = HSMCI_GetItMask(pHw);
	dwMaskedSr = dwSr & dwMsk;

	/* Check errors */
	if (dwMaskedSr & STATUS_ERRORS) {
		if (dwMaskedSr & HSMCI_SR_RTOE)
			pCmd->bStatus = SDMMC_ERROR_NORESPONSE;

		if (pCmd->bCmd != 12) pMcid->bState = MCID_ERROR;

		//pMcid->bState = MCID_ERROR;
	}

	dwMsk &= ~STATUS_ERRORS;

	/* Check command complete */
	if (dwMaskedSr & HSMCI_SR_CMDRDY) {
		TRACE_DEBUG("HSMCI_SR_CMDRDY \r\n");
		HSMCI_DisableIt(pHw, HSMCI_IDR_CMDRDY);
		dwMsk &= ~(uint32_t)HSMCI_IMR_CMDRDY;
	}

	/* Check if not busy */
	if (dwMaskedSr & HSMCI_SR_NOTBUSY) {
		TRACE_DEBUG("NOTBUSY ");
		HSMCI_DisableIt(pHw, HSMCI_IDR_NOTBUSY);
		dwMsk &= ~(uint32_t)HSMCI_IMR_NOTBUSY;
	}

	/* Check if TX ready */
	if (dwMaskedSr & HSMCI_SR_TXRDY) {
		TRACE_DEBUG("TXRDY ");
		dwMsk &= ~(uint32_t)HSMCI_IMR_TXRDY;
	}

	/* Check if FIFO empty (all data sent) */
	if (dwMaskedSr & HSMCI_SR_FIFOEMPTY) {
		/* Disable FIFO empty */
		HSMCI_DisableIt(pHw, HSMCI_IDR_FIFOEMPTY);
		dwMsk &= ~(uint32_t)HSMCI_IMR_FIFOEMPTY;
		TRACE_DEBUG("FIFOEMPTY %x \r\n", dwMsk);
	}

	/* Check if DMA finished */
	if (dwMaskedSr & HSMCI_SR_XFRDONE) {
    pMcid->pXdmad->XdmaChannels[pMcid->dwDmaCh].state = XDMAD_STATE_DONE;
		HSMCI_DisableIt(pHw, HSMCI_IDR_XFRDONE);
		dwMsk &= ~(uint32_t)HSMCI_IMR_XFRDONE;
		TRACE_DEBUG("HSMCI_SR_XFRDONE %x \r\n", dwMsk);
	}

	/* All none error mask done, complete the command */
	if (0 == dwMsk || pMcid->bState == MCID_ERROR) {
		/* Error reset */
		if (pMcid->bState == MCID_ERROR)
			MCI_Reset(pMcid, 1);
		else  {
			pCmd->bStatus = SDMMC_SUCCESS;

			if (pCmd->pResp) {
				uint8_t bRspSize, i;

				switch (pCmd->cmdOp.bmBits.respType) {
				case 1: case 3: case 4: case 5: case 6: case 7:
					bRspSize = 1;
					break;

				case 2:
					bRspSize = 4;
					break;

				default:
					bRspSize = 0;
				}

				for (i = 0; i < bRspSize; i ++)
					pCmd->pResp[i] = HSMCI_GetResponse(pHw);
			}
		}

		/* Disable interrupts */
		HSMCI_DisableIt(pHw, HSMCI_GetItMask(pHw));
		/* Disable peripheral */
		/* Command is finished */
		_FinishCmd(pMcid, pCmd->bStatus);
	}

    Board_SetLed(LED_SD, LED_TRI_B, LED_PRIO_NONE);
}

/**
 * Cancel pending SD/MMC command.
 */
uint32_t MCID_CancelCmd(sMcid *pMcid)
{
	if (pMcid->bState == MCID_IDLE)
		return SDMMC_ERROR_STATE;

	if (pMcid->bState == MCID_CMD) {
		/* Cancel ... */
		MCI_Reset(pMcid, 1);
		/* Command is finished */
		_FinishCmd(pMcid, SDMMC_ERROR_USER_CANCEL);
	}

	return SDMMC_OK;
}

/**
 * Reset MCID and disable HW
 */
void MCID_Reset(sMcid *pMcid)
{
	Hsmci *pHw = pMcid->pMciHw;

	MCID_CancelCmd(pMcid);

	/* Disable */
	MCI_DISABLE(pHw);
	/* MR reset */
	HSMCI_ConfigureMode(pHw, HSMCI_GetMode(pHw) & (HSMCI_MR_CLKDIV_Msk
						| HSMCI_MR_PWSDIV_Msk));
	/* BLKR reset */
	HSMCI_ConfigureTransfer(pHw, 0, 0);

	/* Cancel ... */
	MCI_Reset(pMcid, 1);

	if (pMcid->bState == MCID_CMD) {
		/* Command is finished */
		_FinishCmd(pMcid, SDMMC_ERROR_USER_CANCEL);
	}
}

/**
 * Check if the command is finished
 */
uint32_t MCID_IsCmdCompleted(sMcid *pMcid)
{
	sSdmmcCommand *pCmd = pMcid->pCmd;

	if (pMcid->bPolling)
		MCID_Handler(pMcid);

	if (pMcid->bState == MCID_CMD)
		return 0;

	if (pCmd)
		return 0;

	return 1;
}

/**
 * IO control functions
 */
uint32_t MCID_IOCtrl(sMcid *pMcid, uint32_t bCtl, uint32_t param)
{
	Hsmci *pMciHw = pMcid->pMciHw;
	assert(pMcid);
	assert(pMcid->pMciHw);


	switch (bCtl) {
	case SDMMC_IOCTL_BUSY_CHECK:
		*(uint32_t *)param = !MCID_IsCmdCompleted(pMcid);
		break;

	case SDMMC_IOCTL_POWER:
		return SDMMC_ERROR_NOT_SUPPORT;

	case SDMMC_IOCTL_RESET:
		MCID_Reset(pMcid);
		return SDMMC_SUCCESS;

	case SDMMC_IOCTL_CANCEL_CMD:
		return MCID_CancelCmd(pMcid);

	case SDMMC_IOCTL_SET_CLOCK:
		*(uint32_t *)param = MCI_SetSpeed(pMcid,
										  *(uint32_t *)param,
										  pMcid->dwMck);
		break;

	case SDMMC_IOCTL_SET_HSMODE:
		HSMCI_HsEnable(pMciHw, *(uint32_t *)param);
		*(uint32_t *)param = HSMCI_IsHsEnabled(pMciHw);

		break;

	case SDMMC_IOCTL_SET_BUSMODE:
		HSMCI_SetBusWidth(pMciHw, *(uint32_t *)param);
		break;

	case SDMMC_IOCTL_GET_BUSMODE:
		//*(uint32_t*)param = 8; /* Max 4-bit bus */
		break;

	case SDMMC_IOCTL_GET_HSMODE:
		*(uint32_t *)param = 1; /* Supported */
		break;

	default:
		return SDMMC_ERROR_NOT_SUPPORT;
	}

	return SDMMC_OK;
}
