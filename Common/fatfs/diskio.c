/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

#include "libstoragemedia.h"
#include "MEDSdcard.h"

/* Definitions of physical drive number for each drive */

uint8_t MEDSdcard_Read(
sMedia *media,
uint32_t address,
void* data,
uint32_t length,
MediaCallback callback,
void* argument);

uint8_t MEDSdcard_Write(
sMedia* media,
uint32_t address,
void* data,
uint32_t length,
MediaCallback callback,
void* argument);


sMedia medias[MAX_MEDS];
/** SDCard driver instance. */
COMPILER_SECTION(".data_TCM") COMPILER_ALIGNED(32) sSdCard sdDrv[BOARD_NUM_MCI];


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv) {

	case DEV_MMC:
		stat = 0;
        break;

	}
	return stat;
}


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv) {

	case DEV_MMC:
        if((MEDSdcard_Initialize(&medias[pdrv], &sdDrv[pdrv])))
        {
            stat = RES_OK;
        }
        break;
	}
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res = RES_PARERR;
	int result;

	switch (pdrv) {
	case DEV_MMC :
		{
            unsigned int addr, len;
            if (medias[pdrv].blockSize < SECTOR_SIZE_DEFAULT)
            {
              addr = sector * (SECTOR_SIZE_DEFAULT / medias[pdrv].blockSize);
              len  = count * (SECTOR_SIZE_DEFAULT / medias[pdrv].blockSize);
            }
            else
            {
              addr = sector;
              len  = count;
            }

            result = MEDSdcard_Read(&medias[pdrv], addr, (void*)buff, len, NULL, NULL);

            if( result == MED_STATUS_SUCCESS )
            {
              res = RES_OK;
            }
            else
            {
              TRACE_ERROR("MED_Read pb: 0x%X\r\n", result);
              res = RES_ERROR;
            }
        }
	}
	return res;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res = RES_PARERR;
	int result;
    void* tmp;
    tmp = (void*) buff;

	switch (pdrv) {
	case DEV_MMC :
        {
            unsigned int addr, len;
            if (medias[pdrv].blockSize < SECTOR_SIZE_DEFAULT)
            {
              addr = sector * (SECTOR_SIZE_DEFAULT / medias[pdrv].blockSize);
              len  = count * (SECTOR_SIZE_DEFAULT / medias[pdrv].blockSize);
            }
            else
            {
              addr = sector;
              len  = count;
            }

            result = MEDSdcard_Write(&medias[pdrv], addr, (void*)tmp, len, NULL, NULL);

            if( result == MED_STATUS_SUCCESS )
            {
              res = RES_OK;
            }
            else
            {
              TRACE_ERROR("MED_Write pb: 0x%X\r\n", result);
              res = RES_ERROR;
            }

        }
    }
	return res;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res = RES_PARERR;

	switch (pdrv) {
	case DEV_MMC :
       switch (cmd)
       {
         case GET_BLOCK_SIZE:
         *(DWORD*)buff = 1;
         res = RES_OK;
         break;

         case GET_SECTOR_COUNT :   /* Get number of sectors on the disk (DWORD) */
         if (medias[DEV_MMC].blockSize < SECTOR_SIZE_DEFAULT)
         {
           *(DWORD*)buff = (DWORD)(medias[DEV_MMC].size /
           (SECTOR_SIZE_DEFAULT /
           medias[DEV_MMC].blockSize));
         }
         else
         {
           *(DWORD*)buff = (DWORD)(medias[DEV_MMC].size);
         }
         res = RES_OK;
         break;

         case GET_SECTOR_SIZE :   /* Get sectors on the disk (WORD) */
         if (medias[DEV_MMC].blockSize < SECTOR_SIZE_DEFAULT)
         {
           *(WORD*)buff = SECTOR_SIZE_DEFAULT;
         }
         else
         {
           *(WORD*)buff = medias[DEV_MMC].blockSize;
         }
         res = RES_OK;
         break;

         case CTRL_SYNC :   /* Make sure that data has been written */
         res = RES_OK;
         break;

         default:
         res = RES_PARERR;
       }
       break;
	}

	return res;
}

