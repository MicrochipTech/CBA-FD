/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/* Copyright (c) 2017, Microchip                                                */
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
 *
 *  \section Purpose
 *
 *  Interface for configuring and using Timer Counter (TC) peripherals.
 *
 *  \section Usage
 *  -# Optionally, use TC_FindMckDivisor() to let the program find the best
 *     TCCLKS field value automatically.
 *  -# Configure a Timer Counter in the desired mode using TC_Configure().
 *  -# Start or stop the timer clock using TC_Start() and TC_Stop().
 */

#ifndef _MCAN_
#define _MCAN_

/*------------------------------------------------------------------------------
 *         Headers
 *------------------------------------------------------------------------------*/

#include "sam.h"
#include "mcan_config.h"

#include <stdint.h>

/*------------------------------------------------------------------------------
 *         Global functions
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

#define MCAN0_STD_FLTS_WRDS           (MCAN0_NMBR_STD_FLTS)
/* 128 max filters */
#define MCAN0_EXT_FLTS_WRDS           (MCAN0_NMBR_EXT_FLTS * 2)
/* 64 max filters */
#define MCAN0_RX_FIFO0_WRDS           (MCAN0_NMBR_RX_FIFO0_ELMTS * ((MCAN0_RX_FIFO0_ELMT_SZ/4) + 2))
/* 64 elements max */
#define MCAN0_RX_FIFO1_WRDS           (MCAN0_NMBR_RX_FIFO1_ELMTS * ((MCAN0_RX_FIFO1_ELMT_SZ/4) + 2))
/* 64 elements max */
#define MCAN0_RX_DED_BUFS_WRDS        (MCAN0_NMBR_RX_DED_BUF_ELMTS * ((MCAN0_RX_BUF_ELMT_SZ/4) + 2))
/* 64 elements max */
#define MCAN0_TX_EVT_FIFO_WRDS        (MCAN0_NMBR_TX_EVT_FIFO_ELMTS * 2)
/* 32 elements max */
#define MCAN0_TX_DED_BUF_WRDS         (MCAN0_NMBR_TX_DED_BUF_ELMTS * ((MCAN0_TX_BUF_ELMT_SZ/4) + 2))
/* 32 elements max */
#define MCAN0_TX_FIFO_Q_WRDS          (MCAN0_NMBR_TX_FIFO_Q_ELMTS * ((MCAN0_TX_BUF_ELMT_SZ/4) + 2))
/* 32 elements max */

#define MCAN1_STD_FLTS_WRDS           (MCAN1_NMBR_STD_FLTS)
/* 128 max filters */
#define MCAN1_EXT_FLTS_WRDS           (MCAN1_NMBR_EXT_FLTS * 2)
/* 64 max filters */
#define MCAN1_RX_FIFO0_WRDS           (MCAN1_NMBR_RX_FIFO0_ELMTS * ((MCAN1_RX_FIFO0_ELMT_SZ/4) + 2))
/* 64 elements max */
#define MCAN1_RX_FIFO1_WRDS           (MCAN1_NMBR_RX_FIFO1_ELMTS * ((MCAN1_RX_FIFO1_ELMT_SZ/4) + 2))
/* 64 elements max */
#define MCAN1_RX_DED_BUFS_WRDS        (MCAN1_NMBR_RX_DED_BUF_ELMTS * ((MCAN1_RX_BUF_ELMT_SZ/4) + 2))
/* 64 elements max */
#define MCAN1_TX_EVT_FIFO_WRDS        (MCAN1_NMBR_TX_EVT_FIFO_ELMTS * 2)
/* 32 elements max */
#define MCAN1_TX_DED_BUF_WRDS         (MCAN1_NMBR_TX_DED_BUF_ELMTS * ((MCAN1_TX_BUF_ELMT_SZ/4) + 2))
/* 32 elements max */
#define MCAN1_TX_FIFO_Q_WRDS          (MCAN1_NMBR_TX_FIFO_Q_ELMTS * ((MCAN1_TX_BUF_ELMT_SZ/4) + 2))
/* 32 elements max */

#if (MCAN0_NMBR_STD_FLTS > 128)
	#error "Invalid CAN0 # of Standard Filters"
#endif
#if (MCAN0_NMBR_EXT_FLTS > 64)
	#error "Invalid CAN0 # of Extended Filters"
#endif
#if (MCAN0_NMBR_RX_FIFO0_ELMTS > 64)
	#error "Invalid CAN0 # RX FIFO 0 ELEMENTS"
#endif
#if (MCAN0_NMBR_RX_FIFO1_ELMTS > 64)
	#error "Invalid CAN0 # RX FIFO 0 ELEMENTS"
#endif
#if (MCAN0_NMBR_RX_DED_BUF_ELMTS > 64)
	#error "Invalid CAN0 # RX BUFFER ELEMENTS"
#endif
#if (MCAN0_NMBR_TX_EVT_FIFO_ELMTS > 32)
	#error "Invalid CAN0 # TX EVENT FIFO ELEMENTS"
#endif
#if ((MCAN0_NMBR_TX_DED_BUF_ELMTS + MCAN0_NMBR_TX_FIFO_Q_ELMTS)  > 32)
	#error "Invalid CAN0 # TX BUFFER ELEMENTS"
#endif

#if   (8 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (0u)
#elif (12 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (1u)
#elif (16 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (2u)
#elif (20 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (3u)
#elif (24 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (4u)
#elif (32 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (5u)
#elif (48 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (6u)
#elif (64 == MCAN0_RX_FIFO0_ELMT_SZ)
	#define MCAN0_RX_FIFO0_DATA_SIZE  (7u)
#else
	#error "Invalid CAN0 RX FIFO0 ELEMENT SIZE"
#endif

#if   (8 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (0u)
#elif (12 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (1u)
#elif (16 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (2u)
#elif (20 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (3u)
#elif (24 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (4u)
#elif (32 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (5u)
#elif (48 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (6u)
#elif (64 == MCAN0_RX_FIFO1_ELMT_SZ)
	#define MCAN0_RX_FIFO1_DATA_SIZE  (7u)
#else
	#error "Invalid CAN0 RX FIFO1 ELEMENT SIZE"
#endif

#if   (8 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (0u)
#elif (12 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (1u)
#elif (16 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (2u)
#elif (20 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (3u)
#elif (24 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (4u)
#elif (32 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (5u)
#elif (48 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (6u)
#elif (64 == MCAN0_RX_BUF_ELMT_SZ)
	#define MCAN0_RX_BUF_DATA_SIZE  (7u)
#else
	#error "Invalid CAN0 RX BUFFER ELEMENT SIZE"
#endif

#if   (8 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (0u)
#elif (12 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (1u)
#elif (16 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (2u)
#elif (20 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (3u)
#elif (24 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (4u)
#elif (32 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (5u)
#elif (48 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (6u)
#elif (64 == MCAN0_TX_BUF_ELMT_SZ)
	#define MCAN0_TX_BUF_DATA_SIZE  (7u)
#else
	#error "Invalid CAN0 TX BUFFER ELEMENT SIZE"
#endif

#if (MCAN1_NMBR_STD_FLTS > 128)
	#error "Invalid CAN1 # of Standard Filters"
#endif
#if (MCAN1_NMBR_EXT_FLTS > 64)
	#error "Invalid CAN1 # of Extended Filters"
#endif
#if (MCAN1_NMBR_RX_FIFO0_ELMTS > 64)
	#error "Invalid CAN1 # RX FIFO 0 ELEMENTS"
#endif
#if (MCAN1_NMBR_RX_FIFO1_ELMTS > 64)
	#error "Invalid CAN1 # RX FIFO 0 ELEMENTS"
#endif
#if (MCAN1_NMBR_RX_DED_BUF_ELMTS > 64)
	#error "Invalid CAN1 # RX BUFFER ELEMENTS"
#endif
#if (MCAN1_NMBR_TX_EVT_FIFO_ELMTS > 32)
	#error "Invalid CAN1 # TX EVENT FIFO ELEMENTS"
#endif
#if ((MCAN1_NMBR_TX_DED_BUF_ELMTS + MCAN1_NMBR_TX_FIFO_Q_ELMTS)  > 32)
	#error "Invalid CAN1 # TX BUFFER ELEMENTS"
#endif

#if   (8 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (0u)
#elif (12 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (1u)
#elif (16 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (2u)
#elif (20 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (3u)
#elif (24 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (4u)
#elif (32 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (5u)
#elif (48 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (6u)
#elif (64 == MCAN1_RX_FIFO0_ELMT_SZ)
	#define MCAN1_RX_FIFO0_DATA_SIZE  (7u)
#else
	#error "Invalid CAN1 RX FIFO0 ELEMENT SIZE"
#endif

#if   (8 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (0u)
#elif (12 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (1u)
#elif (16 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (2u)
#elif (20 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (3u)
#elif (24 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (4u)
#elif (32 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (5u)
#elif (48 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (6u)
#elif (64 == MCAN1_RX_FIFO1_ELMT_SZ)
	#define MCAN1_RX_FIFO1_DATA_SIZE  (7u)
#else
	#error "Invalid CAN1 RX FIFO1 ELEMENT SIZE"
#endif

#if   (8 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (0u)
#elif (12 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (1u)
#elif (16 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (2u)
#elif (20 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (3u)
#elif (24 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (4u)
#elif (32 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (5u)
#elif (48 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (6u)
#elif (64 == MCAN1_RX_BUF_ELMT_SZ)
	#define MCAN1_RX_BUF_DATA_SIZE  (7u)
#else
	#error "Invalid CAN1 RX BUFFER ELEMENT SIZE"
#endif

#if   (8 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (0u)
#elif (12 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (1u)
#elif (16 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (2u)
#elif (20 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (3u)
#elif (24 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (4u)
#elif (32 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (5u)
#elif (48 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (6u)
#elif (64 == MCAN1_TX_BUF_ELMT_SZ)
	#define MCAN1_TX_BUF_DATA_SIZE  (7u)
#else
	#error "Invalid CAN1 TX BUFFER ELEMENT SIZE"
#endif

#define CAN_11_BIT_ID_MASK                 (0x7FF)
#define CAN_29_BIT_ID_MASK                 (0x1FFFFFFF)
  
typedef enum {
	CAN_STD_ID = 0,
	CAN_EXT_ID = 1
} MCan_IdType;

typedef enum {
	CAN_DLC_0 = 0,
	CAN_DLC_1 = 1,
	CAN_DLC_2 = 2,
	CAN_DLC_3 = 3,
	CAN_DLC_4 = 4,
	CAN_DLC_5 = 5,
	CAN_DLC_6 = 6,
	CAN_DLC_7 = 7,
	CAN_DLC_8 = 8,
	CAN_DLC_12 = 9,
	CAN_DLC_16 = 10,
	CAN_DLC_20 = 11,
	CAN_DLC_24 = 12,
	CAN_DLC_32 = 13,
	CAN_DLC_48 = 14,
	CAN_DLC_64 = 15
} MCan_DlcType;

typedef enum {
	CAN_FIFO_0 = 0,
	CAN_FIFO_1 = 1
} MCan_FifoType;

typedef enum {
	CAN_INTR_LINE_0 = 0,
	CAN_INTR_LINE_1 = 1
} MCan_IntrLineType;

#define CAN_FLAG_STANDARD (0u << 0u)
#define CAN_FLAG_EXTENDED (1u << 0u)
#define CAN_FLAG_FD       (1u << 1u)
#define CAN_FLAG_BRS      (1u << 2u)

typedef struct MailboxInfoTag {
	uint32_t   id;
	uint32_t   length;
	uint32_t   timestamp;
    uint32_t   flags;
    uint32_t   messageMarker;
} MailboxInfoType;


typedef struct MailBox8Tag {
	MailboxInfoType info;
	uint8_t         data[8];
} Mailbox8Type;

typedef struct MailBox12Tag {
	MailboxInfoType info;
	uint8_t         data[12];
} Mailbox12Type;

typedef struct MailBox16Tag {
	MailboxInfoType info;
	uint8_t         data[16];
} Mailbox16Type;

typedef struct MailBox20Tag {
	MailboxInfoType info;
	uint8_t         data[20];
} Mailbox20Type;

typedef struct MailBox24Tag {
	MailboxInfoType info;
	uint8_t         data[24];
} Mailbox24Type;

typedef struct MailBox32Tag {
	MailboxInfoType info;
	uint8_t         data[32];
} Mailbox32ype;

typedef struct MailBox48Tag {
	MailboxInfoType info;
	uint8_t         data[48];
} Mailbox48Type;

typedef struct MailBox64Tag {
	MailboxInfoType info;
	uint8_t         data[64];
} Mailbox64Type;

typedef struct MCan_MsgRamPntrsTag {
	uint32_t *pStdFilts;
	uint32_t *pExtFilts;
	uint32_t *pRxFifo0;
	uint32_t *pRxFifo1;
	uint32_t *pRxDedBuf;
	uint32_t *pTxEvtFifo;
	uint32_t *pTxDedBuf;
	uint32_t *pTxFifoQ;
} MCan_MsgRamPntrs;

typedef struct MCan_ConfigTag {
    uint32_t          instanceId;
	Mcan             *pMCan;
	uint32_t          bitTiming;
	uint32_t          fastBitTiming;
	uint32_t          nmbrStdFilts;
	uint32_t          nmbrExtFilts;
	uint32_t          nmbrFifo0Elmts;
	uint32_t          nmbrFifo1Elmts;
	uint32_t          nmbrRxDedBufElmts;
	uint32_t          nmbrTxEvtFifoElmts;
	uint32_t          nmbrTxDedBufElmts;
	uint32_t          nmbrTxFifoQElmts;
	uint32_t          rxFifo0ElmtSize;
	uint32_t          rxFifo1ElmtSize;
	uint32_t          rxBufElmtSize;
	// Element sizes and data sizes (encoded element size)
	uint32_t          txBufElmtSize;
	// Element size and data size (encoded element size)
	MCan_MsgRamPntrs  msgRam;
} MCan_ConfigType;

extern const MCan_ConfigType mcan0Config;
extern const MCan_ConfigType mcan1Config;

__STATIC_INLINE uint32_t MCAN_IsTxComplete(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	return (mcan->MCAN_IR & MCAN_IR_TC);
}

__STATIC_INLINE void MCAN_ClearTxComplete(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	mcan->MCAN_IR = MCAN_IR_TC;
}

__STATIC_INLINE uint32_t MCAN_IsMessageStoredToRxDedBuffer(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;

	return (mcan->MCAN_IR & MCAN_IR_DRX);
}

__STATIC_INLINE void MCAN_ClearMessageStoredToRxBuffer(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	mcan->MCAN_IR = MCAN_IR_DRX;
}

__STATIC_INLINE uint32_t MCAN_IsMessageStoredToRxFifo0(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	return (mcan->MCAN_IR & MCAN_IR_RF0N);
}

__STATIC_INLINE void MCAN_ClearMessageStoredToRxFifo0(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	mcan->MCAN_IR = MCAN_IR_RF0N;
}

__STATIC_INLINE uint32_t MCAN_IsMessageStoredToRxFifo1(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	return (mcan->MCAN_IR & MCAN_IR_RF1N);
}

__STATIC_INLINE void MCAN_ClearMessageStoredToRxFifo1(
	const MCan_ConfigType *mcanConfig)
{
	Mcan *mcan = mcanConfig->pMCan;
	mcan->MCAN_IR = MCAN_IR_RF1N;
}

__STATIC_INLINE uint8_t MCAN_GetFillLevelFifo0(Mcan* mcan)
{
  return (uint8_t) (mcan->MCAN_RXF0S & MCAN_RXF0S_F0FL_Msk) >> MCAN_RXF0S_F0FL_Pos;
}

__STATIC_INLINE uint8_t MCAN_GetFillLevelFifo1(Mcan* mcan)
{
  return (uint8_t) (mcan->MCAN_RXF1S & MCAN_RXF1S_F1FL_Msk) >> MCAN_RXF1S_F1FL_Pos;
}

__STATIC_INLINE uint8_t MCAN_GetFillLevelTxEventFifo(Mcan* mcan)
{
  return (uint8_t) (mcan->MCAN_TXEFS & MCAN_TXEFS_EFFL_Msk) >> MCAN_TXEFS_EFFL_Pos;
}

__STATIC_INLINE uint8_t MCAN_GetFreeLevelTxEventFifo(Mcan* mcan)
{
  return 
    ((mcan->MCAN_TXEFC & MCAN_TXEFC_EFS_Msk) >> MCAN_TXEFC_EFS_Pos) -
    ((mcan->MCAN_TXEFS & MCAN_TXEFS_EFFL_Msk) >> MCAN_TXEFS_EFFL_Pos);
}

__STATIC_INLINE uint8_t MCAN_GetHeadFifo0(Mcan* mcan)
{
  /* Head is retrieved from the put pointer, which points to the next (free) element after head */
  uint8_t head = (mcan->MCAN_RXF0S & MCAN_RXF0S_F0PI_Msk) >> MCAN_RXF0S_F0PI_Pos;
  if(head == 0)
  {
    head = MCAN0_NMBR_RX_FIFO0_ELMTS - 1u;
  }
  else
  {
    head--;
  }
  return head;
}

__STATIC_INLINE uint8_t MCAN_GetHeadFifoEvent(Mcan* mcan)
{
  /* Head is retrieved from the put pointer, which points to the next (free) element after head */
  uint8_t head = (mcan->MCAN_TXEFS & MCAN_TXEFS_EFPI_Msk) >> MCAN_TXEFS_EFPI_Pos;
  if(head == 0)
  {
    head = MCAN0_NMBR_TX_EVT_FIFO_ELMTS - 1u;
  }
  else
  {
    head--;
  }
  return head;
}

__STATIC_INLINE uint8_t MCAN_GetTailFifo0(Mcan* mcan)
{
  return (mcan->MCAN_RXF0S & MCAN_RXF0S_F0GI_Msk) >> MCAN_RXF0S_F0GI_Pos;
}

__STATIC_INLINE uint8_t MCAN_GetTailFifoEvent(Mcan* mcan)
{
  return (mcan->MCAN_TXEFS & MCAN_TXEFS_EFGI_Msk) >> MCAN_TXEFS_EFGI_Pos;
}

__STATIC_INLINE void MCAN_SetModeNormal(Mcan* mcan)
{
  mcan->MCAN_CCCR &= ~(MCAN_CCCR_TEST | MCAN_CCCR_MON);
  mcan->MCAN_TEST = 0u;
}

__STATIC_INLINE void MCAN_SetModeListenOnly(Mcan* mcan)
{
  mcan->MCAN_CCCR |= MCAN_CCCR_MON;
  mcan->MCAN_TEST = 0u;
}

__STATIC_INLINE void MCAN_SetModeLoopbackInt(Mcan* mcan)
{
  mcan->MCAN_CCCR |= MCAN_CCCR_TEST | MCAN_CCCR_MON;
  mcan->MCAN_TEST = MCAN_TEST_LBCK;
}

__STATIC_INLINE void MCAN_SetModeLoopbackExt(Mcan* mcan)
{
  mcan->MCAN_CCCR |= MCAN_CCCR_TEST;
  mcan->MCAN_TEST = MCAN_TEST_LBCK;
}

__STATIC_INLINE void MCAN_Enable(Mcan* mcan)
{
  mcan->MCAN_CCCR = (mcan->MCAN_CCCR & ~MCAN_CCCR_INIT_ENABLED);
}

void MCAN_EnterInitMode(Mcan* mcan);

void MCAN_Init(
	const MCan_ConfigType *mcanConfig);

void MCAN_InitFdEnable(
	const MCan_ConfigType *mcanConfig);

void MCAN_InitFdBitRateSwitchEnable(
	const MCan_ConfigType *mcanConfig);

void MCAN_InitTxQueue(
	const MCan_ConfigType *mcanConfig);

void MCAN_InitLoopback(
	const MCan_ConfigType *mcanConfig);

void MCAN_RequestIso11898_1(
	const MCan_ConfigType *mcanConfig);

void MCAN_RequestFd(
	const MCan_ConfigType *mcanConfig);

void MCAN_RequestFdBitRateSwitch(
	const MCan_ConfigType *mcanConfig);

void MCAN_IEnableMessageStoredToRxDedBuffer(
	const MCan_ConfigType *mcanConfig,
	MCan_IntrLineType line);

uint8_t   *MCAN_ConfigTxDedBuffer(
	const MCan_ConfigType *mcanConfig,
	uint8_t buffer,
	uint32_t id,
	MCan_IdType idType,
	MCan_DlcType dlc);

void MCAN_SendTxDedBuffer(
	const MCan_ConfigType *mcanConfig,
	uint8_t buffer,
	uint8_t mode);

uint32_t MCAN_AddToTxFifoQ(
	const MCan_ConfigType *mcanConfig,
	uint32_t id, MCan_IdType idType,
	MCan_DlcType dlc, const uint8_t *data, uint8_t mode);

uint8_t MCAN_IsBufferTxd(
	const MCan_ConfigType *mcanConfig,
	uint8_t buffer);

void MCAN_ConfigRxBufferFilter(
	const MCan_ConfigType *mcanConfig,
	uint32_t buffer,
	uint32_t filter,
	uint32_t id,
	MCan_IdType idType);

void MCAN_ConfigRxClassicFilter(
	const MCan_ConfigType *mcanConfig,
	MCan_FifoType fifo,
	uint8_t filter,
	uint32_t id,
	MCan_IdType idType,
	uint32_t mask);

uint8_t MCAN_IsNewDataInRxDedBuffer(
	const MCan_ConfigType *mcanConfig,
	uint8_t buffer);

void MCAN_GetRxDedBuffer(
	const MCan_ConfigType *mcanConfig,
	uint8_t buffer,
	Mailbox64Type *pRxMailbox);

uint32_t MCAN_GetRxFifoBuffer(
	const MCan_ConfigType *mcanConfig,
	MCan_FifoType fifo,
	Mailbox64Type *pRxMailbox);

void MCAN_ConfigRxFifoFilter(const MCan_ConfigType *mcanConfig,
	MCan_FifoType fifo, uint8_t filter, uint32_t id,
	MCan_IdType idType, uint32_t mask);


uint32_t MCAN_GetTxEventFifoBuffer(const MCan_ConfigType* mcanConfig, Mailbox64Type* pTxEvent);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MCAN_ */

