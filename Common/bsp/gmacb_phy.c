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

#include "board.h"
#include "gmacb_phy.h"
#include "gmii.h"
#include "pio.h"
#include <FreeRTOS.h>
#include <task.h>

/*---------------------------------------------------------------------------
 *         Definitions
 *---------------------------------------------------------------------------*/

/** Default max retry count */
#define GMACB_RETRY_MAX            30000

#ifndef TRACE_ERROR
#define TRACE_ERROR(...) { }
#endif

#ifndef TRACE_DEBUG
#define TRACE_DEBUG(...) { }
#endif

#ifndef TRACE_INFO
#define TRACE_INFO(...) { }
#endif

/*---------------------------------------------------------------------------
 *         Local functions
 *---------------------------------------------------------------------------*/


/**
 * Wait PHY operation complete.
 * Return 1 if the operation completed successfully.
 * May be need to re-implemented to reduce CPU load.
 * \param retry: the retry times, 0 to wait forever until complete.
 */
static uint8_t GMACB_WaitPhy(Gmac *pHw, uint32_t retry)
{
  volatile uint32_t retry_count = 0;

  while (!GMAC_IsIdle(pHw))
  {
    if (retry == 0) continue;

    retry_count++;
    if (retry_count >= retry)
      return 0;
	}

	return 1;
}

/**
 * Read PHY register.
 * Return 1 if successfully, 0 if timeout.
 * \param pHw HW controller address
 * \param PhyAddress PHY Address
 * \param Address Register Address
 * \param pValue Pointer to a 32 bit location to store read data
 * \param retry The retry times, 0 to wait forever until complete.
 */
static uint8_t GMACB_ReadPhy(Gmac *pHw,
							 uint8_t PhyAddress,
							 uint8_t Address,
							 uint16_t *pValue,
							 uint32_t retry)
{
  if(GMAC_PHYMaintain(pHw, PhyAddress, Address, 1, 0) == BSP_OK)
  {
    if(GMACB_WaitPhy(pHw, retry) == 0) {
      TRACE_ERROR("TimeOut GMACB_ReadPhy\r\n");
      return 0;
    }
    
    if(GMAC_PHYData(pHw, pValue) == BSP_OK)
    {
      return 1;
    }
  }
  return 0;
}

/**
 * Write PHY register
 * Return 1 if successfully, 0 if timeout.
 * \param pHw HW controller address
 * \param PhyAddress PHY Address
 * \param Address Register Address
 * \param Value Data to write (Actually 16 bit data)
 * \param retry The retry times, 0 to wait forever until complete.
 */
static uint8_t GMACB_WritePhy(Gmac *pHw,
							  uint8_t PhyAddress,
							  uint8_t Address,
							  uint16_t Value,
							  uint32_t retry)
{
  if(GMAC_PHYMaintain(pHw, PhyAddress, Address, 0, Value) == BSP_OK)
  {
    if(GMACB_WaitPhy(pHw, retry) == 0)
    {
      TRACE_ERROR("TimeOut GMACB_WritePhy\r\n");
      return 0;
    }
    return 1;
  }
  return 0;
}

/*---------------------------------------------------------------------------
 *         Exported functions
 *---------------------------------------------------------------------------*/

/**
 * \brief Find a valid PHY Address (from 0 to 31).
 * \param pMacb Pointer to the MACB instance
 * \return 0xFF when no valid PHY Address found.
 */
#ifdef USE_DEPRECATED
static uint8_t GMACB_FindValidPhy(GMacb *pMacb)
{
	sGmacd *pDrv = pMacb->pGmacd;
	Gmac *pHw = pDrv->pHw;

	uint32_t  retryMax;
	uint16_t  value = 0;
	uint8_t rc;
	uint8_t phyAddress;
	uint8_t cnt;

	TRACE_DEBUG("GMACB_FindValidPhy\r\n");

	phyAddress = pMacb->phyAddress;
	retryMax = pMacb->retryMax;

	/* Check current phyAddress */
	rc = phyAddress;

	if (GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID1R, &value, retryMax) == 0)
		TRACE_ERROR("GMACB PROBLEM\r\n");

	TRACE_DEBUG("_PHYID1  : 0x%hX, addr: %d\r\n", value, phyAddress);

	/* Find another one */
	if (value != PHY_ID_KSZ8061_MSB) {
		rc = 0xFF;

		for (cnt = 0; cnt < 32; cnt ++) {
			phyAddress = (phyAddress + 1) & 0x1F;

			if (GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID1R, &value, retryMax)
				 == 0)
				TRACE_ERROR("MACB PROBLEM\r\n");

			TRACE_DEBUG("_PHYID1  : 0x%hX, addr: %d\r\n", value, phyAddress);

			if (value == PHY_ID_KSZ8061_MSB) {

				rc = phyAddress;
				break;
			}
		}
	}

	if (rc != 0xFF) {
		TRACE_INFO("** Valid PHY Found: %d\r\n", rc);
		GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID1R, &value, retryMax);
		TRACE_DEBUG("_PHYID1R  : 0x%hX, addr: %d\r\n", value, phyAddress);
		GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID2R, &value, retryMax);
		TRACE_DEBUG("_EMSR  : 0x%hX, addr: %d\r\n", value, phyAddress);
	}

	return rc;
}
#endif

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/


/**
 * \brief Dump all the useful registers.
 * \param pMacb          Pointer to the MACB instance
 */
void GMACB_DumpRegisters(GMacb *pMacb)
{
	sGmacd *pDrv = pMacb->pGmacd;
	Gmac *pHw = pDrv->pHw;

	uint8_t phyAddress;
	uint32_t retryMax;
	uint16_t value;

	TRACE_INFO("GMACB_DumpRegisters\r\n");

	phyAddress = pMacb->phyAddress;
	retryMax = pMacb->retryMax;

	TRACE_INFO("GMII MACB @ %d) Registers:\r\n", phyAddress);

	GMACB_ReadPhy(pHw, phyAddress, GMII_BMCR, &value, retryMax);
	TRACE_INFO(" _BMCR     : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_BMSR, &value, retryMax);
	TRACE_INFO(" _BMSR     : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID1R, &value, retryMax);
	TRACE_INFO(" _PHYID1     : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID2R, &value, retryMax);
	TRACE_INFO(" _PHYID2     : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ANAR, &value, retryMax);
	TRACE_INFO(" _ANAR     : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ANLPAR, &value, retryMax);
	TRACE_INFO(" _ANLPAR   : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ANER, &value, retryMax);
	TRACE_INFO(" _ANER     : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ANNPR, &value, retryMax);
	TRACE_INFO(" _ANNPR    : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ANLPNPAR, &value, retryMax);
	TRACE_INFO(" _ANLPNPAR : 0x%hX\r\n", (unsigned)value);

	TRACE_INFO(" \r\n");

	GMACB_ReadPhy(pHw, phyAddress, GMII_RXERCR, &value, retryMax);
	TRACE_INFO(" _RXERCR   : 0x%hX\r\n", (unsigned)value);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ICSR, &value, retryMax);
	TRACE_INFO(" _ICSR     : 0x%hX\r\n", (unsigned)value);
	TRACE_INFO(" \r\n");
}

/**
 * \brief Setup the maximum timeout count of the driver.
 * \param pMacb Pointer to the MACB instance
 * \param toMax Timeout maximum count.
 */
void GMACB_SetupTimeout(GMacb *pMacb, uint32_t toMax)
{
	pMacb->retryMax = toMax;
}

/**
 * \brief Initialize the MACB instance.
 * \param pMacb Pointer to the MACB instance
 * \param phyAddress   The PHY address used to access the PHY
 */
void GMACB_Init(GMacb *pMacb, sGmacd *pGmacd, uint8_t phyAddress)
{
	pMacb->pGmacd = pGmacd;
	pMacb->phyAddress = phyAddress;
	/* Initialize timeout by default */
	pMacb->retryMax = GMACB_RETRY_MAX;
}


/**
 * \brief Issue a SW reset to reset all registers of the PHY.
 * \param pMacb Pointer to the MACB instance
 * \return 1 if successfully, 0 if timeout.
 */
uint8_t GMACB_ResetPhy(GMacb *pMacb)
{
	sGmacd *pDrv = pMacb->pGmacd;
	Gmac *pHw = pDrv->pHw;
	uint32_t retryMax;
	uint16_t bmcr = GMII_RESET;
	uint8_t phyAddress;
	uint32_t timeout = 10;
	uint8_t ret = 1;

	TRACE_INFO(" GMACB_ResetPhy\r\n");

	phyAddress = pMacb->phyAddress;
	retryMax = pMacb->retryMax;

	bmcr = GMII_RESET;
	GMACB_WritePhy(pHw, phyAddress, GMII_BMCR, bmcr, retryMax);

	do {
		GMACB_ReadPhy(pHw, phyAddress, GMII_BMCR, &bmcr, retryMax);
		timeout--;
	} while ((bmcr & GMII_RESET) && timeout);

	if (!timeout)
		ret = 0;

	return (ret);
}

/**
 * \brief Do a HW initialize to the PHY (via RSTC) and set up clocks & PIOs
 * This should be called only once to initialize the PHY pre-settings.
 * The PHY address is reset status of CRS,RXD[3:0] (the emacPins' pullups).
 * The COL pin is used to select MII mode on reset (pulled up for Reduced MII)
 * The RXDV pin is used to select test mode on reset (pulled up for test mode)
 * The above pins should be predefined for corresponding settings in resetPins
 * The GMAC peripheral pins are configured after the reset done.
 * \param pMacb Pointer to the MACB instance
 * \param mck         Main clock setting to initialize clock
 * \param resetPins   Pointer to list of PIOs to configure before HW RESET
 *                       (for PHY power on reset configuration latch)
 * \param nbResetPins Number of PIO items that should be configured
 * \param emacPins    Pointer to list of PIOs for the EMAC interface
 * \param nbEmacPins  Number of PIO items that should be configured
 * \return 1 if RESET OK, 0 if timeout.
 */
uint8_t GMACB_InitPhy(GMacb *pMacb,
					  uint32_t mck)
{
  sGmacd *pDrv = pMacb->pGmacd;
  Gmac *pHw = pDrv->pHw;
  uint8_t rc = 1;

  /* Perform RESET */
  TRACE_DEBUG("RESET PHY\r\n");

  rc = GMAC_SetMdcClock(pHw, mck);
  GMAC_EnableMdio(pHw);

  if (rc > 0) {
    TRACE_ERROR("No Valid MDC clock\r\n");
    return rc;
  }

  rc = GMACB_ResetPhy(pMacb);
  if (!rc) {
    TRACE_ERROR("No Valid MDC clock\r\n");
    return rc;
  }
  
  vTaskDelay(10);

  uint16_t phyId1 = 0;
  uint16_t phyId2 = 0;

  GMACB_ReadPhy(pHw, pMacb->phyAddress, GMII_PHYID1R, &phyId1, pMacb->retryMax);
  GMACB_ReadPhy(pHw, pMacb->phyAddress, GMII_PHYID2R, &phyId2, pMacb->retryMax);

  if(phyId1 == PHY_ID_KSZ8061_MSB)
  {
    GMACB_WritePhy(pHw, pMacb->phyAddress, GMII_ICSR, 0x0, pMacb->retryMax);

    if(phyId2 == PHY_ID_KSZ8061_LSB_Rev3)
    {
    }
  }

  return rc;
}

/**
 * \brief Issue a Auto Negotiation of the PHY
 * \param pMacb Pointer to the MACB instance
 * \return 1 if successfully, 0 if timeout.
 */
uint8_t GMACB_AutoNegotiate(GMacb *pMacb)
{
	sGmacd *pDrv = pMacb->pGmacd;
	Gmac *pHw = pDrv->pHw;
	uint32_t retryMax;
	uint16_t value;
	uint16_t phyAnar;
	uint16_t phyAnalpar;
	uint32_t retryCount = 0;
	uint8_t phyAddress;
	uint8_t rc = 1;
	uint32_t duplex, speed;
	phyAddress = pMacb->phyAddress;
	retryMax = pMacb->retryMax;

	if (!GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID1R, &value, retryMax)) {
		TRACE_ERROR("Pb GEMAC_ReadPhy Id1\r\n");
		rc = 0;
		goto AutoNegotiateExit;
	}

	TRACE_DEBUG("ReadPhy Id1 0x%X, address: %d\r\n", value, phyAddress);

	if (!GMACB_ReadPhy(pHw, phyAddress, GMII_PHYID2R, &phyAnar, retryMax)) {
		TRACE_ERROR("Pb GMACB_ReadPhy Id2\r\n");
		rc = 0;
		goto AutoNegotiateExit;
	}

	TRACE_DEBUG("ReadPhy Id2 0x%X\r\n", phyAnar);

	if ((value == PHY_ID_KSZ8061_MSB)
		 && (((phyAnar) & (~GMII_LSB_MASK)) == PHY_ID_KSZ8061_LSB_Rev3)) {
		TRACE_DEBUG("Vendor Number Model = 0x%X\r\n", ((phyAnar >> 4) & 0x3F));
		TRACE_DEBUG("Model Revision Number = 0x%X\r\n", (phyAnar & 0xF));
	} else
		TRACE_ERROR("Problem OUI value\r\n");

	/** Set the Auto_negotiation Advertisement Register, MII advertising for
	Next page 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3 */
	rc  = GMACB_ReadPhy(pHw, phyAddress, GMII_ANAR, &phyAnar, retryMax);

	if (rc == 0)
		goto AutoNegotiateExit;

	phyAnar = GMII_TX_FDX | GMII_TX_HDX |
			  GMII_10_FDX | GMII_10_HDX | GMII_AN_IEEE_802_3;
	rc = GMACB_WritePhy(pHw, phyAddress, GMII_ANAR, phyAnar, retryMax);

	if (rc == 0)
		goto AutoNegotiateExit;

	/* Read & modify control register */
	rc  = GMACB_ReadPhy(pHw, phyAddress, GMII_BMCR, &value, retryMax);

	if (rc == 0)
		goto AutoNegotiateExit;

	/* Check AutoNegotiate complete */
	value |=  GMII_AUTONEG | GMII_RESTART_AUTONEG;
	rc = GMACB_WritePhy(pHw, phyAddress, GMII_BMCR, value, retryMax);

	if (rc == 0)
		goto AutoNegotiateExit;

	TRACE_DEBUG(" _BMCR: 0x%X\r\n", value);

	// Check AutoNegotiate complete
	while (1) {
 		rc  = GMACB_ReadPhy(pHw, phyAddress, GMII_BMSR, &value, retryMax);

		if (rc == 0) {
			TRACE_ERROR("rc==0\r\n");
			goto AutoNegotiateExit;
		}

		/* Done successfully */
		if (value & GMII_AUTONEG_COMP) {
			TRACE_INFO("AutoNegotiate complete\r\n");
			break;
		}

		/* Timeout check */
		if (retryMax) {
			if (++ retryCount >= retryMax) {
				GMACB_DumpRegisters(pMacb);
				TRACE_ERROR("TimeOut\r\n");
				rc = 0;
				goto AutoNegotiateExit;
			}
		}
	}

	/*Set local link mode */
	while (1) {
		rc  = GMACB_ReadPhy(pHw, phyAddress, GMII_ANLPAR, &phyAnalpar, retryMax);

		if (rc == 0)
			goto AutoNegotiateExit;

		/* Set up the GMAC link speed */
		if ((phyAnar & phyAnalpar) & GMII_TX_FDX) {
			/* set RGMII for 1000BaseTX and Full Duplex */
			duplex = GMAC_DUPLEX_FULL;
			speed = GMAC_SPEED_100M;
			break;
		} else if ((phyAnar & phyAnalpar) & GMII_10_FDX) {
			/* set RGMII for 1000BaseT and Half Duplex*/
			duplex = GMAC_DUPLEX_FULL;
			speed = GMAC_SPEED_10M;
			break;
		} else if ((phyAnar & phyAnalpar) & GMII_TX_HDX) {
			/* set RGMII for 100BaseTX and half Duplex */
			duplex = GMAC_DUPLEX_HALF;
			speed = GMAC_SPEED_100M;
			break;
		} else if ((phyAnar & phyAnalpar) & GMII_10_HDX) {
			// set RGMII for 10BaseT and half Duplex
			duplex = GMAC_DUPLEX_HALF;
			speed = GMAC_SPEED_10M;
			break;
		}
	}

	TRACE_INFO("GMAC_EnableRGMII duplex %u, speed %u\r\n", (unsigned)duplex,
			   (unsigned)speed);

	GMACB_ReadPhy(pHw, phyAddress, GMII_PC1R, &value, retryMax);
	GMACB_ReadPhy(pHw, phyAddress, GMII_PC2R, &value, retryMax);
	GMACB_ReadPhy(pHw, phyAddress, GMII_ICSR, &value, retryMax);
	/* Set up GMAC mode  */
	GMAC_EnableRGMII(pHw, duplex, speed);

AutoNegotiateExit:

	return rc;
}
