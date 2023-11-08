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

#pragma once


#include <sam.h>
#include <xdmac.h>
#include <assert.h>


/*----------------------------------------------------------------------------
 *        Consts
 *----------------------------------------------------------------------------*/
#define XDMAD_TRANSFER_MEMORY  0xFF   /**< DMA transfer from or to memory */
#define XDMAD_ALLOC_FAILED     0xFFFF /**< Channel allocate failed */

/* XDMA_MBR_UBC */
#define XDMA_UBC_NDE (0x1u << 24)
#define   XDMA_UBC_NDE_FETCH_DIS (0x0u << 24)
#define   XDMA_UBC_NDE_FETCH_EN  (0x1u << 24)
#define XDMA_UBC_NSEN (0x1u << 25)
#define   XDMA_UBC_NSEN_UNCHANGED (0x0u << 25)
#define   XDMA_UBC_NSEN_UPDATED (0x1u << 25)
#define XDMA_UBC_NDEN (0x1u << 26)
#define   XDMA_UBC_NDEN_UNCHANGED (0x0u << 26)
#define   XDMA_UBC_NDEN_UPDATED (0x1u << 26)
#define XDMA_UBC_NVIEW_Pos 27
#define    XDMA_UBC_NVIEW_Msk (0x3u << XDMA_UBC_NVIEW_Pos)
#define    XDMA_UBC_NVIEW_NDV0 (0x0u << XDMA_UBC_NVIEW_Pos)
#define    XDMA_UBC_NVIEW_NDV1 (0x1u << XDMA_UBC_NVIEW_Pos)
#define    XDMA_UBC_NVIEW_NDV2 (0x2u << XDMA_UBC_NVIEW_Pos)
#define    XDMA_UBC_NVIEW_NDV3 (0x3u << XDMA_UBC_NVIEW_Pos)


/** DMA status or return code */
typedef enum _XdmadStatus {
	XDMAD_OK = 0,        /**< Operation is successful */
	XDMAD_PARTIAL_DONE,
	XDMAD_DONE,
	XDMAD_BUSY,          /**< Channel occupied or transfer not finished */
	XDMAD_ERROR,         /**< Operation failed */
	XDMAD_CANCELED       /**< Operation cancelled */
} eXdmadStatus, eXdmadRC;

/** DMA state for channel */
typedef enum _XdmadState {
	XDMAD_STATE_FREE = 0,      /**< Free channel */
	XDMAD_STATE_ALLOCATED,     /**< Allocated to some peripheral */
	XDMAD_STATE_START,         /**< DMA started */
	XDMAD_STATE_IN_XFR,        /**< DMA in transferring */
	XDMAD_STATE_DONE,          /**< DMA transfer done */
	XDMAD_STATE_HALTED,        /**< DMA transfer stopped */
} eXdmadState;

/** DMA Programming state for channel */
typedef enum _XdmadProgState {
	XDMAD_SINGLE = 0,
	XDMAD_MULTI,
	XDMAD_LLI,
} eXdmadProgState;

typedef enum _DmaEndpoints {
  DMA_HSMCI_TX = 0,
  DMA_HSMCI_RX = 0,
  DMA_SPI0_TX =  1,
  DMA_SPI0_RX = 2,
  DMA_SPI1_TX = 3,
  DMA_SPI1_RX = 4,
  DMA_QSPI_TX = 5,
  DMA_QSPI_RX = 6,
  DMA_USART0_TX = 7,
  DMA_USART0_RX = 8,
  DMA_USART1_TX = 9,
  DMA_USART1_RX = 10,
  DMA_USART2_TX = 11,
  DMA_USART2_RX = 12,
  DMA_PWM0_TX = 13,
  DMA_TWIHS0_TX = 14,
  DMA_TWIHS0_RX = 15,
  DMA_TWIHS1_TX = 16,
  DMA_TWIHS1_RX = 17,
  DMA_TWIHS2_TX = 18,
  DMA_TWIHS2_RX = 19,
  DMA_UART0_TX = 20,
  DMA_UART0_RX = 21,
  DMA_UART1_TX = 22,
  DMA_UART1_RX = 23,
  DMA_UART2_TX = 24,
  DMA_UART2_RX = 25,
  DMA_UART3_TX = 26,
  DMA_UART3_RX = 27,
  DMA_UART4_TX = 28,
  DMA_UART4_RX = 29,
  DMA_DACC_TX = 30,
  DMA_DACC_RX = 31,
  DMA_SSC_TX = 32,
  DMA_SSC_RX = 33,
  DMA_PIOA_TX = 34,
  DMA_AFEC0_TX = 35,
  DMA_AFEC1_RX = 36,
  DMA_AES_TX = 37,
  DMA_AES_RX = 38,
  DMA_PWM1_TX = 39,
  DMA_TC0_TX = 40,
  DMA_TC1_TX = 41,
  DMA_TC2_TX = 42,
  DMA_TC3_TX = 43,
  DMA_I2SC0_TX_L = 44,
  DMA_I2SC0_RX_L = 45,
  DMA_I2SC1_TX_L = 46,
  DMA_I2SC1_RX_L = 47,
  DMA_I2SC0_TX_R = 48,
  DMA_I2SC0_RX_R = 49,
  DMA_I2SC1_TX_R = 50,
  DMA_I2SC1_RX_R = 51
} eDmaEndpoints;

/** DMA transfer callback */
typedef void (*XdmadTransferCallback)(uint32_t Channel, void *pArg);

/** DMA driver channel */
typedef struct _XdmadChannel {
	XdmadTransferCallback fCallback; /**< Callback */
	void *pArg;                     /**< Callback argument */
	volatile eXdmadState state;         /**< DMA channel state */
} sXdmadChannel;

/** DMA driver instance */
typedef struct _Xdmad {
	Xdmac *pXdmacs;
	sXdmadChannel XdmaChannels[XDMACCHID_NUMBER];
	uint8_t  pollingMode;
} sXdmad;

typedef struct _XdmadCfg {
	/** Microblock Control Member. */
	uint32_t mbr_ubc;
	/** Source Address Member. */
	uint32_t mbr_sa;
	/** Destination Address Member. */
	uint32_t mbr_da;
	/** Configuration Register. */
	uint32_t mbr_cfg;
	/** Block Control Member. */
	uint32_t mbr_bc;
	/** Data Stride Member. */
	uint32_t mbr_ds;
	/** Source Microblock Stride Member. */
	uint32_t mbr_sus;
	/** Destination Microblock Stride Member. */
	uint32_t mbr_dus;
} sXdmadCfg;

/** \brief Structure for storing parameters for DMA view0 that can be
 * performed by the DMA Master transfer.*/
typedef struct _LinkedListDescriporView0 {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t mbr_ubc;
	/** Transfer Address Member. */
	uint32_t mbr_ta;
} LinkedListDescriporView0;

/** \brief Structure for storing parameters for DMA view1 that can be
 * performed by the DMA Master transfer.*/
typedef struct _LinkedListDescriporView1 {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t mbr_ubc;
	/** Source Address Member. */
	uint32_t mbr_sa;
	/** Destination Address Member. */
	uint32_t mbr_da;
} LinkedListDescriporView1;

/** \brief Structure for storing parameters for DMA view2 that can be
 * performed by the DMA Master transfer.*/
typedef struct _LinkedListDescriporView2 {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t mbr_ubc;
	/** Source Address Member. */
	uint32_t mbr_sa;
	/** Destination Address Member. */
	uint32_t mbr_da;
	/** Configuration Register. */
	uint32_t mbr_cfg;
} LinkedListDescriporView2;

/** \brief Structure for storing parameters for DMA view3 that can be
 * performed by the DMA Master transfer.*/
typedef struct _LinkedListDescriporView3 {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t mbr_ubc;
	/** Source Address Member. */
	uint32_t mbr_sa;
	/** Destination Address Member. */
	uint32_t mbr_da;
	/** Configuration Register. */
	uint32_t mbr_cfg;
	/** Block Control Member. */
	uint32_t mbr_bc;
	/** Data Stride Member. */
	uint32_t mbr_ds;
	/** Source Microblock Stride Member. */
	uint32_t mbr_sus;
	/** Destination Microblock Stride Member. */
	uint32_t mbr_dus;
} LinkedListDescriporView3;

extern void XDMAD_Initialize(sXdmad *pXdmad, uint8_t bPollingMode);
extern void XDMAD_Handler(sXdmad *pDmad);
extern uint32_t XDMAD_AllocateChannel(sXdmad *pXdmad);
extern eXdmadRC XDMAD_FreeChannel(sXdmad *pXdmad, uint32_t dwChannel);
extern eXdmadRC XDMAD_ConfigureTransfer(sXdmad *pXdmad, uint32_t dwChannel, sXdmadCfg *pXdmaParam, uint32_t dwXdmaDescCfg, uint32_t dwXdmaDescAddr, uint32_t dwXdmaIntEn);
extern eXdmadRC XDMAD_PrepareChannel(sXdmad *pXdmad, uint32_t dwChannel);
extern eXdmadRC XDMAD_IsTransferDone(sXdmad *pXdmad, uint32_t dwChannel);
extern eXdmadRC XDMAD_StartTransfer(sXdmad *pXdmad, uint32_t dwChannel);
extern eXdmadRC XDMAD_SetCallback(sXdmad *pXdmad, uint32_t dwChannel, XdmadTransferCallback fCallback, void *pArg);
extern eXdmadRC XDMAD_StopTransfer(sXdmad *pXdmad, uint32_t dwChannel);
