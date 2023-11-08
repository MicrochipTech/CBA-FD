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

/** \addtogroup xdmad_module
 *
 * \section Xdma xDma Configuration Usage
 *
 * To configure a XDMA channel, the user has to follow these few steps :
 * <ul>
 * <li> Initialize a XDMA driver instance by XDMAD_Initialize().</li>
 * <li> choose an available (disabled) channel using XDMAD_AllocateChannel().</li>
 * <li> After the XDMAC selected channel has been programmed,
 * XDMAD_PrepareChannel() is to enable clock and dma peripheral of the DMA, and
 * set Configuration register to set up the transfer type (memory or non-memory
 * peripheral for source and destination) and flow control device.</li>
 * <li> Invoke XDMAD_StartTransfer() to start DMA transfer  or
 * XDMAD_StopTransfer() to force stop DMA transfer.</li>
 * <li> Once the buffer of data is transferred, XDMAD_IsTransferDone()
 * checks if DMA transfer is finished.</li>
 * <li> XDMAD_Handler() handles XDMA interrupt, and invoking XDMAD_SetCallback()
 * if provided.</li>
 * </ul>
 *
 * Related files:\n
 * \ref xdmad.h\n
 * \ref xdmad.c.\n
 */

/** \file */

/** \addtogroup dmad_functions
  @{*/

/*----------------------------------------------------------------------------
 *        Includes
 *----------------------------------------------------------------------------*/

#include <stddef.h>
#include <xdmad.h>
#include <pmc.h>
static uint8_t xDmad_Initialized = 0;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Initialize xDMA driver instance.
 * \param pXdmad Pointer to xDMA driver instance.
 * \param bPollingMode Polling DMA transfer:
 *                     1. Via XDMAD_IsTransferDone(); or
 *                     2. Via XDMAD_Handler().
 */
void XDMAD_Initialize(sXdmad *pXdmad, uint8_t bPollingMode)
{
  uint32_t j;

  if(xDmad_Initialized != 0) {
    return;
  }

  PMC_EnablePeripheral(ID_XDMAC);

  /* V71 only has a single XDMA instance */
  pXdmad->pXdmacs = XDMAC;
  pXdmad->pollingMode = bPollingMode;

  /* Number of channels is derived from auto generated system header */
  for (j = 0; j < XDMACCHID_NUMBER; j ++)
  {
    pXdmad->XdmaChannels[j].fCallback = 0;
    pXdmad->XdmaChannels[j].pArg      = 0;
    pXdmad->XdmaChannels[j].state = XDMAD_STATE_FREE;
  }

  xDmad_Initialized = 1;
}


/**
 * \brief Allocate a XDMA channel for upper layer.
 * \param pXdmad  Pointer to xDMA driver instance.
 * \return Channel number if allocation successful, return
 * XDMAD_ALLOC_FAILED if allocation failed.
 */
uint32_t XDMAD_AllocateChannel(sXdmad *pXdmad)
{
  uint32_t i=0;
  uint32_t ret = XDMAD_ALLOC_FAILED;

  __disable_irq();

  for(i=0; i<XDMACCHID_NUMBER; i++)
  {
    if(pXdmad->XdmaChannels[i].state == XDMAD_STATE_FREE)
    {
      /* Allocate the channel */
      pXdmad->XdmaChannels[i].state = XDMAD_STATE_ALLOCATED;

      ret = i;
      break;
    }
  }

  __enable_irq();
  
  return ret;
}

/**
 * \brief Free the specified xDMA channel.
 * \param pXdmad     Pointer to xDMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eXdmadRC XDMAD_FreeChannel(sXdmad *pXdmad, uint32_t dwChannel)
{
  if(dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }

  switch(pXdmad->XdmaChannels[dwChannel].state)
  {
  case XDMAD_STATE_ALLOCATED:
  case XDMAD_STATE_START:
  case XDMAD_STATE_IN_XFR:
    return XDMAD_BUSY;

  case XDMAD_STATE_DONE:
  case XDMAD_STATE_HALTED:
    pXdmad->XdmaChannels[dwChannel].state = XDMAD_STATE_FREE;
  case XDMAD_STATE_FREE:
    break;
  }

  return XDMAD_OK;
}

/**
 * \brief Set the callback function for xDMA channel transfer.
 * \param pXdmad     Pointer to xDMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 * \param fCallback Pointer to callback function.
 * \param pArg Pointer to optional argument for callback.
 */
eXdmadRC XDMAD_SetCallback(sXdmad *pXdmad, uint32_t dwChannel, XdmadTransferCallback fCallback, void *pArg)
{
  if(dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }

  if(pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_FREE) {
    return XDMAD_ERROR;
  }      
  else if(pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_START) {
    return XDMAD_BUSY;
  }

  pXdmad->XdmaChannels[dwChannel].fCallback = fCallback;
  pXdmad->XdmaChannels[dwChannel].pArg = pArg;

  return XDMAD_OK;
}


/**
 * \brief Enable clock of the xDMA peripheral, Enable the dma peripheral,
 * configure configuration register for xDMA transfer.
 * \param pXdmad     Pointer to xDMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 * \param dwCfg     Configuration value.
 */
eXdmadRC XDMAD_PrepareChannel(sXdmad *pXdmad, uint32_t dwChannel)
{
  Xdmac *pXdmac = pXdmad->pXdmacs;

  if(dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }

  if(pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_FREE) {
    return XDMAD_ERROR;
  }
  else if((pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_START) ||
          (pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_IN_XFR)) {
    return XDMAD_BUSY;
  }

  /* Clear dummy status */
  XDMAC_GetChannelIsr(pXdmac, dwChannel);
  
  /* Disables XDMAC interrupt for the given channel. */
  XDMAC_DisableGIt (pXdmac, dwChannel);
  XDMAC_DisableChannelIt (pXdmac, dwChannel, 0xFF);
  
  /* Disable the given dma channel. */
  XDMAC_DisableChannel(pXdmac, dwChannel);
  XDMAC_SetSourceAddr(pXdmac, dwChannel, 0);
  XDMAC_SetDestinationAddr(pXdmac, dwChannel, 0);
  XDMAC_SetBlockControl(pXdmac, dwChannel, 0);
  XDMAC_SetChannelConfig(pXdmac, dwChannel, 0);
  XDMAC_SetDescriptorAddr(pXdmac, dwChannel, 0, 0);
  XDMAC_SetDescriptorControl(pXdmac, dwChannel, 0);
  
  return XDMAD_OK;
}

/**
 * \brief xDMA interrupt handler
 * \param pxDmad Pointer to DMA driver instance.
 */
void XDMAD_Handler(sXdmad *pDmad)
{
  sXdmadChannel *pCh;
  uint32_t xdmaChannelIntStatus, xdmaGlobaIntStatus, xdmaGlobalChStatus;
  uint8_t bExec = 0;
  uint8_t _iChannel;
  Xdmac *pXdmac = pDmad->pXdmacs;

  xdmaGlobaIntStatus = XDMAC_GetGIsr(pXdmac);

  if(xdmaGlobaIntStatus != 0)
  {
    xdmaGlobalChStatus = XDMAC_GetGlobalChStatus(pXdmac);         // Get a mask of all enabled channels

    for(_iChannel=0; _iChannel<XDMACCHID_NUMBER; _iChannel++)     // Walk through all channels
    {
      if((xdmaGlobaIntStatus & (1u << _iChannel)) == 0) {         // Check if there is a channel interrupt pending
        continue;
      }

      pCh = &pDmad->XdmaChannels[_iChannel];                      // Select channel

      if(pCh->state == XDMAD_STATE_FREE) {                        // Check if channel is still allocated
        continue;
      }
            
      bExec = 0;

      if((xdmaGlobalChStatus & (XDMAC_GS_ST0 << _iChannel)) == 0)
      {
        xdmaChannelIntStatus = XDMAC_GetMaskChannelIsr(pXdmac, _iChannel);

        if(xdmaChannelIntStatus & XDMAC_CIS_BIS)                                  // Block transfer has finished
        {
          if((XDMAC_GetChannelItMask(pXdmac, _iChannel) & XDMAC_CIM_LIM) == 0) {  // Linked list is disabled
            pCh->state = XDMAD_STATE_DONE;                                        // Transaction is finished!
            bExec = 1;
          }
        }

        if(xdmaChannelIntStatus & XDMAC_CIS_LIS) {                                // Linked list transfer has finished
          pCh->state = XDMAD_STATE_DONE;                                          // Transaction is finished!
          bExec = 1;
        }

        if(xdmaChannelIntStatus & XDMAC_CIS_DIS) {                                // DMA channel has been disabled
          pCh->state = XDMAD_STATE_DONE;                                          // Synchronized transaction will finish, memory-memory transactions are aborted
          bExec = 1;
        }
      }
      else
      {
        /* Block end interrupt for LLI dma mode */
        if(XDMAC_GetChannelIsr(pXdmac, _iChannel) & XDMAC_CIS_BIS)
        {
          /* Execute callback */
          if(pCh->fCallback) {
            pCh->fCallback(_iChannel, pCh->pArg);
          }
        }
      }

      /* Execute callback */
      if (bExec && pCh->fCallback) {
        pCh->fCallback(_iChannel, pCh->pArg);
      }
    }
  }
}

/**
 * \brief Check if DMA transfer is finished.
 *        In polling mode XDMAD_Handler() is polled.
 * \param pDmad     Pointer to DMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eXdmadRC XDMAD_IsTransferDone(sXdmad *pXdmad, uint32_t dwChannel)
{
  uint8_t state;
  if(dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }

  state = pXdmad->XdmaChannels[dwChannel].state;

  if(state == XDMAD_STATE_ALLOCATED) {
    return XDMAD_OK;
  }

  if(state == XDMAD_STATE_FREE) {
    return XDMAD_ERROR;
  }
  else if(state != XDMAD_STATE_DONE) {
    if(pXdmad->pollingMode) {
      XDMAD_Handler(pXdmad);
    }
    return XDMAD_BUSY;
  }

  return XDMAD_OK;
}


/**
 * \brief Configure DMA for a single transfer.
 * \param pXdmad     Pointer to xDMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eXdmadRC XDMAD_ConfigureTransfer(sXdmad *pXdmad, uint32_t dwChannel, sXdmadCfg *pXdmaParam, uint32_t dwXdmaDescCfg, uint32_t dwXdmaDescAddr, uint32_t dwXdmaIntEn)
{
  if(dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }

  Xdmac *pXdmac = pXdmad->pXdmacs;
  XDMAC_GetChannelIsr(pXdmac, dwChannel);

  if(pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_FREE) {
    return XDMAD_ERROR;
  }

  if(pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_START) {
    return XDMAD_BUSY;
  }

  if((dwXdmaDescCfg & XDMAC_CNDC_NDE_Msk) == XDMAC_CNDC_NDE_DSCR_FETCH_EN)
  {
    /* Linked List is enabled */
    if((dwXdmaDescCfg & XDMAC_CNDC_NDVIEW_Msk) == XDMAC_CNDC_NDVIEW_NDV0)
    {
      XDMAC_SetChannelConfig(pXdmac, dwChannel, pXdmaParam->mbr_cfg);
      XDMAC_SetSourceAddr(pXdmac, dwChannel, pXdmaParam->mbr_sa);
      XDMAC_SetDestinationAddr(pXdmac, dwChannel, pXdmaParam->mbr_da);
    }

    if((dwXdmaDescCfg & XDMAC_CNDC_NDVIEW_Msk) == XDMAC_CNDC_NDVIEW_NDV1) {
      XDMAC_SetChannelConfig(pXdmac, dwChannel, pXdmaParam->mbr_cfg);
    }

    XDMAC_SetDescriptorAddr(pXdmac, dwChannel, dwXdmaDescAddr, 0);
    XDMAC_SetDescriptorControl(pXdmac, dwChannel, dwXdmaDescCfg);
    XDMAC_DisableChannelIt(pXdmac, dwChannel, XDMAC_CID_Msk);
    XDMAC_EnableChannelIt(pXdmac, dwChannel, dwXdmaIntEn);
  }
  else
  {
    /* LLI is disabled. */
    XDMAC_SetSourceAddr(pXdmac, dwChannel, pXdmaParam->mbr_sa);
    XDMAC_SetDestinationAddr(pXdmac, dwChannel, pXdmaParam->mbr_da);
    XDMAC_SetMicroblockControl(pXdmac, dwChannel, pXdmaParam->mbr_ubc);
    XDMAC_SetBlockControl(pXdmac, dwChannel, pXdmaParam->mbr_bc);
    XDMAC_SetDataStride_MemPattern(pXdmac, dwChannel, pXdmaParam->mbr_ds);
    XDMAC_SetSourceMicroBlockStride(pXdmac, dwChannel, pXdmaParam->mbr_sus);
    XDMAC_SetDestinationMicroBlockStride(pXdmac, dwChannel, pXdmaParam->mbr_dus);
    XDMAC_SetChannelConfig(pXdmac, dwChannel, pXdmaParam->mbr_cfg);
    XDMAC_SetDescriptorAddr(pXdmac, dwChannel, 0, 0);
    XDMAC_SetDescriptorControl(pXdmac, dwChannel, 0);
    XDMAC_EnableChannelIt (pXdmac, dwChannel, dwXdmaIntEn);
  }

  return XDMAD_OK;
}

/**
 * \brief Start xDMA transfer.
 * \param pXdmad     Pointer to XDMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eXdmadRC XDMAD_StartTransfer(sXdmad *pXdmad, uint32_t dwChannel)
{
  if (dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }
  
  Xdmac *pXdmac = pXdmad->pXdmacs;

  if (pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_FREE) {
    return XDMAD_ERROR;
  }
  else if (pXdmad->XdmaChannels[dwChannel].state == XDMAD_STATE_START) {
    return XDMAD_BUSY;
  }

  /* Change state to transferring */
  pXdmad->XdmaChannels[dwChannel].state = XDMAD_STATE_START;
  XDMAC_EnableChannel(pXdmac, dwChannel);

  if(pXdmad->pollingMode == 0) {
    XDMAC_EnableGIt(pXdmac, dwChannel);
  }

  return XDMAD_OK;
}


/**
 * \brief Stop DMA transfer.
 * \param pDmad     Pointer to DMA driver instance.
 * \param dwChannel ControllerNumber << 8 | ChannelNumber.
 */
eXdmadRC XDMAD_StopTransfer(sXdmad *pXdmad, uint32_t dwChannel)
{
  if (dwChannel >= XDMACCHID_NUMBER) {
    return XDMAD_ERROR;
  }

  Xdmac *pXdmac = pXdmad->pXdmacs;
  pXdmad->XdmaChannels[dwChannel].state = XDMAD_STATE_HALTED;

  /* Disable channel */
  XDMAC_DisableChannel(pXdmac, dwChannel);
  /* Disable interrupts */
  XDMAC_DisableChannelIt(pXdmac, dwChannel, XDMAC_CID_Msk);
  /* Clear pending status */
  XDMAC_GetChannelIsr(pXdmac, dwChannel);
  XDMAC_GetGlobalChStatus(pXdmac);
  
  return XDMAD_OK;
}
