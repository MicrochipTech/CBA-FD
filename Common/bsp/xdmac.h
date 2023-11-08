/*                                                                              */
/*                                                                              */
/* © 2016 Microchip Technology Inc. and its subsidiaries.                       */
/*                                                                              */
/* Subject to your compliance with these terms, you may use Microchip software  */
/* and any derivatives exclusively with Microchip products. It is your          */
/* responsibility to comply with third party license terms applicable to your   */
/* use of third party software (including open source software) that may        */
/* accompany Microchip software.                                                */
/*                                                                              */
/* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER       */
/* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED */
/* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A           */
/* PARTICULAR PURPOSE.                                                          */
/*                                                                              */
/* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,    */
/* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND        */
/* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS    */
/* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE       */
/* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN  */
/* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, */
/* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.                  */
/*                                                                              */
/*                                                                              */

#pragma once


#include <sam.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Get XDMAC global type.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetType(Xdmac *pXdmac)
{
  return pXdmac->XDMAC_GTYPE;
}

/**
 * \brief Get XDMAC global configuration.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetConfig(Xdmac *pXdmac)
{
  return pXdmac->XDMAC_GCFG;
}

/**
 * \brief Get XDMAC global weighted arbiter configuration.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetArbiter(Xdmac *pXdmac)
{
  return pXdmac->XDMAC_GWAC;
}

/**
 * \brief Enables XDMAC global interrupt.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param dwInteruptMask IT to be enabled.
 */
static inline void XDMAC_EnableGIt (Xdmac *pXdmac, uint8_t dwInteruptMask)
{
  pXdmac->XDMAC_GIE = (XDMAC_GIE_IE0 << dwInteruptMask);
}

/**
 * \brief Disables XDMAC global interrupt
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param dwInteruptMask IT to be enabled
 */
static inline void XDMAC_DisableGIt(Xdmac *pXdmac, uint8_t dwInteruptMask)
{
  pXdmac->XDMAC_GID = (XDMAC_GID_ID0 << dwInteruptMask);
}

/**
 * \brief Get XDMAC global interrupt mask.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetGItMask(Xdmac *pXdmac)
{
  return (pXdmac->XDMAC_GIM);
}

/**
 * \brief Get XDMAC global interrupt status.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetGIsr(Xdmac *pXdmac)
{
  return (pXdmac->XDMAC_GIS & XDMAC_GIS_IS_Msk);
}

/**
 * \brief Get XDMAC masked global interrupt.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetMaskedGIsr(Xdmac *pXdmac)
{
  uint32_t _dwStatus;
  _dwStatus = pXdmac->XDMAC_GIS;
  _dwStatus &= pXdmac->XDMAC_GIM;
  return _dwStatus;
}

/**
 * \brief enables the relevant channel of given XDMAC.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_EnableChannel(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GE = (XDMAC_GE_EN0 << channel);
}

/**
 * \brief enables the relevant channels of given XDMAC.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param bmChannels Channels bitmap.
 */
static inline void XDMAC_EnableChannels(Xdmac *pXdmac, uint32_t bmChannels)
{
  pXdmac->XDMAC_GE = bmChannels;
}

/**
 * \brief Disables the relevant channel of given XDMAC.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_DisableChannel(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GD = (XDMAC_GD_DI0 << channel);
}

/**
 * \brief Disables the relevant channels of given XDMAC.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param bmChannels Channels bitmap.
 */
static inline void XDMAC_DisableChannels(Xdmac *pXdmac, uint32_t bmChannels)
{
  pXdmac->XDMAC_GD = bmChannels;
}

/**
 * \brief Get Global channel status of given XDMAC.
 * \note: When set to 1, this bit indicates that the channel x is enabled.
   If a channel disable request is issued, this bit remains asserted
   until pending transaction is completed.
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetGlobalChStatus(Xdmac *pXdmac)
{
  return pXdmac->XDMAC_GS;
}

/**
 * \brief Suspend the relevant channel's read.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_SuspendReadChannel(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GRS |= XDMAC_GRS_RS0 << channel;
}

/**
 * \brief Suspend the relevant channel's write.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_SuspendWriteChannel(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GWS |= XDMAC_GWS_WS0 << channel;
}

/**
 * \brief Suspend the relevant channel's read & write.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_SuspendReadWriteChannel(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GRWS = (XDMAC_GRWS_RWS0 << channel);
}

/**
 * \brief Resume the relevant channel's read & write.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_ResumeReadWriteChannel(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GRWR = (XDMAC_GRWR_RWR0 << channel);
}

/**
 * \brief Set software transfer request on the relevant channel.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_SoftwareTransferReq(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GSWR = (XDMAC_GSWR_SWREQ0 << channel);
}

/**
 * \brief Get software transfer status of the relevant channel.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 */
static inline uint32_t XDMAC_GetSoftwareTransferStatus(Xdmac *pXdmac)
{
  return pXdmac->XDMAC_GSWS;
}

/**
 * \brief Set software flush request on the relevant channel.
 * \note: This API is used as polling without enabling FIE interrupt.
 * The user can use it in interrupt mode after deleting while sentense.
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline void XDMAC_SoftwareFlushReq(Xdmac *pXdmac, uint8_t channel)
{
  pXdmac->XDMAC_GSWF = (XDMAC_GSWF_SWF0 << channel);
 // while (!(XDMAC_GetChannelIsr(pXdmac, channel) & XDMAC_CIS_FIS));
}

/**
 * \brief Disable interrupt with mask on the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param dwInteruptMask Interrupt mask.
 */
static inline void XDMAC_EnableChannelIt(Xdmac *pXdmac, uint8_t channel, uint8_t dwInteruptMask)
{
  pXdmac->XdmacChid[channel].XDMAC_CIE = dwInteruptMask;
}

/**
 * \brief Enable interrupt with mask on the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param dwInteruptMask Interrupt mask.
 */
static inline void XDMAC_DisableChannelIt(Xdmac *pXdmac, uint8_t channel, uint8_t dwInteruptMask)
{
  pXdmac->XdmacChid[channel].XDMAC_CID = dwInteruptMask;
}

/**
 * \brief Get interrupt mask for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline uint32_t XDMAC_GetChannelItMask(Xdmac *pXdmac, uint8_t channel)
{
  return pXdmac->XdmacChid[channel].XDMAC_CIM;
}

/**
 * \brief Get interrupt status for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline uint32_t XDMAC_GetChannelIsr(Xdmac *pXdmac, uint8_t channel)
{
  return pXdmac->XdmacChid[channel].XDMAC_CIS;
}

/**
 * \brief Get masked interrupt status for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline uint32_t XDMAC_GetMaskChannelIsr(Xdmac *pXdmac, uint8_t channel)
{
  uint32_t status;
  status = pXdmac->XdmacChid[channel].XDMAC_CIS;
  status &= pXdmac->XdmacChid[channel].XDMAC_CIM;
  return status;
}

/**
 * \brief Set source address for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param addr Source address.
 */
static inline void XDMAC_SetSourceAddr(Xdmac *pXdmac, uint8_t channel, uint32_t addr)
{
  pXdmac->XdmacChid[channel].XDMAC_CSA = addr;
}

/**
 * \brief Set destination address for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param addr Destination address.
 */
static inline void XDMAC_SetDestinationAddr(Xdmac *pXdmac, uint8_t channel, uint32_t addr)
{
  pXdmac->XdmacChid[channel].XDMAC_CDA = addr;
}

/**
 * \brief Set next descriptor's address & interface for the relevant channel of
 *  given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param addr Address of next descriptor.
 * \param ndaif Interface of next descriptor.
 */
static inline void XDMAC_SetDescriptorAddr(Xdmac *pXdmac, uint8_t channel, uint32_t addr, uint8_t ndaif)
{
  pXdmac->XdmacChid[channel].XDMAC_CNDA = (addr & 0xFFFFFFFCu) | (ndaif & 0x1u);
}

/**
 * \brief Set next descriptor's configuration for the relevant channel of
 *  given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param config Configuration of next descriptor.
 */
static inline void XDMAC_SetDescriptorControl(Xdmac *pXdmac, uint8_t channel, uint8_t config)
{
  pXdmac->XdmacChid[channel].XDMAC_CNDC = config;
}

/**
 * \brief Set microblock length for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param ublen Microblock length.
 */
static inline void XDMAC_SetMicroblockControl(Xdmac *pXdmac, uint8_t channel, uint32_t ublen)
{
  pXdmac->XdmacChid[channel].XDMAC_CUBC = XDMAC_CUBC_UBLEN(ublen);
}

/**
 * \brief Set block length for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param blen Block length.
 */
static inline void XDMAC_SetBlockControl(Xdmac *pXdmac, uint8_t channel, uint16_t blen)
{
  pXdmac->XdmacChid[channel].XDMAC_CBC = XDMAC_CBC_BLEN(blen);
}

/**
 * \brief Set configuration for the relevant channel of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param config Channel configuration.
 */
static inline void XDMAC_SetChannelConfig(Xdmac *pXdmac, uint8_t channel, uint32_t config)
{
  pXdmac->XdmacChid[channel].XDMAC_CC = config;
}

/**
 * \brief Get the relevant channel's configuration of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline uint32_t XDMAC_GetChannelConfig(Xdmac *pXdmac, uint8_t channel)
{
  return pXdmac->XdmacChid[channel].XDMAC_CC;
}

/**
 * \brief Set the relevant channel's data stride memory pattern of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param dds_msp Data stride memory pattern.
 */
static inline void XDMAC_SetDataStride_MemPattern(Xdmac *pXdmac, uint8_t channel, uint32_t dds_msp)
{
  pXdmac->XdmacChid[channel].XDMAC_CDS_MSP = dds_msp;
}

/**
 * \brief Set the relevant channel's source microblock stride of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param subs Source microblock stride.
 */
static inline void XDMAC_SetSourceMicroBlockStride(Xdmac *pXdmac, uint8_t channel, uint32_t subs)
{
  pXdmac->XdmacChid[channel].XDMAC_CSUS = XDMAC_CSUS_SUBS(subs);
}

/**
 * \brief Set the relevant channel's destination microblock stride of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 * \param dubs Destination microblock stride.
 */
static inline void XDMAC_SetDestinationMicroBlockStride(Xdmac *pXdmac, uint8_t channel, uint32_t dubs)
{
  pXdmac->XdmacChid[channel].XDMAC_CDUS = XDMAC_CDUS_DUBS(dubs);
}

/**
 * \brief Get the relevant channel's destination address of given XDMA.
 *
 * \param pXdmac Pointer to the XDMAC peripheral.
 * \param channel Particular channel number.
 */
static inline uint32_t XDMAC_GetChDestinationAddr(Xdmac *pXdmac, uint8_t channel)
{
  return pXdmac->XdmacChid[channel].XDMAC_CDA;
}

#ifdef __cplusplus
}
#endif
