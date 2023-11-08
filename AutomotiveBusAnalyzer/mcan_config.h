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

#ifndef _MCAN_CONFIG_
#define _MCAN_CONFIG_

/*------------------------------------------------------------------------------
 *         Global functions
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

#define MCAN0_NMBR_STD_FLTS           2   /* 128 max filters */
#define MCAN0_NMBR_EXT_FLTS           2   /* 64 max filters */
#define MCAN0_NMBR_RX_FIFO0_ELMTS     64  /* # of elements, 64 elements max */
#define MCAN0_NMBR_RX_FIFO1_ELMTS     0   /* # of elements, 64 elements max */
#define MCAN0_NMBR_RX_DED_BUF_ELMTS   0   /* # of elements, 64 elements max */
#define MCAN0_NMBR_TX_EVT_FIFO_ELMTS  32  /* # of elements, 32 elements max */
#define MCAN0_NMBR_TX_DED_BUF_ELMTS   0   /* # of elements, 32 elements max */
#define MCAN0_NMBR_TX_FIFO_Q_ELMTS    32  /* # of elements, 32 elements max */
#define MCAN0_RX_FIFO0_ELMT_SZ        64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */
#define MCAN0_RX_FIFO1_ELMT_SZ        64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */
#define MCAN0_RX_BUF_ELMT_SZ          64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */
#define MCAN0_TX_BUF_ELMT_SZ          64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */

#define MCAN1_NMBR_STD_FLTS           2   /* 128 max filters */
#define MCAN1_NMBR_EXT_FLTS           2   /* 64 max filters */
#define MCAN1_NMBR_RX_FIFO0_ELMTS     64  /* # of elements, 64 elements max */
#define MCAN1_NMBR_RX_FIFO1_ELMTS     0   /* # of elements, 64 elements max */
#define MCAN1_NMBR_RX_DED_BUF_ELMTS   0   /* # of elements, 64 elements max */
#define MCAN1_NMBR_TX_EVT_FIFO_ELMTS  32  /* # of elements, 32 elements max */
#define MCAN1_NMBR_TX_DED_BUF_ELMTS   0   /* # of elements, 32 elements max */
#define MCAN1_NMBR_TX_FIFO_Q_ELMTS    32  /* # of elements, 32 elements max */
#define MCAN1_RX_FIFO0_ELMT_SZ        64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */
#define MCAN1_RX_FIFO1_ELMT_SZ        64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */
#define MCAN1_RX_BUF_ELMT_SZ          64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */
#define MCAN1_TX_BUF_ELMT_SZ          64  /* 8, 12, 16, 20, 24, 32, 48, 64 bytes */

#define MCAN_TX_EVENT_ELMT_SZ         8  /* Tx Fifo element has fixed size */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MCAN_CONFIG_ */

