#include <stdio.h>

#ifdef __GNUC__
#include <unistd.h>
#endif

#ifdef _WIN32
#include <windows.h>
#else
#define Sleep(x) sleep(x)
#endif

#include <string.h>
#include "compiler_w.h"
#include "Bldr_Protocol.h"
#include "Bootloader_flash.h"
#include "CRC.h"

#define IFLASH_PAGE_SIZE    ( 512U)
#define IFLASH_NB_OF_PAGES  (1024U)

uint32_t transferBlob(libusb_device_handle* dev_handle, uint8_t* readBuffer, uint32_t blobSize, uint32_t flashAddress)
{
	int rc = 0;
	struct sbAction action;
	struct sbReply reply;
	int32_t transferred = 0;

	action.magic = MAGIC_ACTION;
	action.address = 0;
	action.checksum = 0;
	action.action = SENDDATA;
	memcpy(action.data, &readBuffer[0], sizeof(action.data));
	rc = libusb_bulk_transfer(
		dev_handle,
		1,
		(uint8_t*)&action,
		sizeof(action),
		&transferred,
		1000);
	if (rc != 0) {
		printf("Send error: %d\r\n", rc);
		return rc;
	}

	action.address = flashAddress;
	action.action = WRITEDATA;
	action.checksum = crc16(readBuffer, blobSize);
	memcpy(action.data, &readBuffer[sizeof(action.data)], sizeof(action.data));
	rc = libusb_bulk_transfer(
		dev_handle,
		1,
		(uint8_t*)&action,
		sizeof(action),
		&transferred,
		1000);
	if (rc != 0) {
		printf("Send error: %d\r\n", rc);
		return rc;
	}

	rc = libusb_bulk_transfer(
		dev_handle,
		0x82,
		(uint8_t*)&reply,
		sizeof(reply),
		&transferred,
		1000);
	if (rc != 0) {
		printf("Recv error: %d\r\n", rc);
		return rc;
	}

	if (reply.action != ERROR_OK)
	{
		printf("Bootloader failed to verify transfer checksum\r\n");
		return 1;
	}
	
	return 0;
}

int sendFirmware(char* file, uint32_t flashAddress, libusb_device_handle* dev_handle)
{
	struct sbAction action;
	struct sbReply reply;
	FILE* stream = 0;
	uint32_t fileSize = 0;
	uint32_t bytesLeft = 0;
	uint32_t offset = 0;
	uint32_t bytesRead = 0;
	uint8_t readBuffer[IFLASH_PAGE_SIZE];
	uint32_t CRC32 = CRC32_INITVAL;
	int rc = 0;
	int transferred = 0;

	printf("Start sendFirmware\r\n");

	crc32c_init_table();

	action.magic = MAGIC_ACTION;
	action.action = QUERYVERSION;
	rc = libusb_bulk_transfer(dev_handle, 1, (uint8_t*)&action, sizeof(action), &transferred, 1000);
	if (rc != 0) {
		printf("QUERYVERSION send error: %d\r\n", rc);
		return -100;
	}
	
	rc = libusb_bulk_transfer(dev_handle, 0x82, (uint8_t*)&reply, sizeof(reply), &transferred, 2000);
	if (rc != 0) {
		printf("QUERYVERSION recv error: %d\r\n", rc);
		return -101;
	}
	
	if (reply.reserved != BOOTLDR_VERSION_1) {
        printf("Unsupported bootloader!\r\n");
        return -102;
	}

#ifdef __GNUC__
	if (stream = fopen(file, "rb")) {
#else
	if (0 == fopen_s(&stream, file, "rb")) {
#endif

		fseek(stream, 0, SEEK_END);
		fileSize = ftell(stream);
		fseek(stream, 0, SEEK_SET);
		action.action = ERASESTART;
		action.address = 0x00420000U;
		rc = libusb_bulk_transfer(dev_handle, 1, (uint8_t*)&action, sizeof(action), &transferred, 1000);
		if (rc != 0) {
			printf("Send error: %d\r\n", rc);
			fclose(stream);
			return -103;
		}

		action.action = ERASEFLASH;
		action.address += /*fileSize + IFLASH_PAGE_SIZE*/ 0x001E0000;
		rc = libusb_bulk_transfer(dev_handle, 1, (uint8_t*)&action, sizeof(action), &transferred, 1000);
		if (rc != 0) {
			printf("Send error: %d\r\n", rc);
			fclose(stream);
			return -104;
		}

		rc = libusb_bulk_transfer(dev_handle, 0x82, (uint8_t*)&reply, sizeof(reply), &transferred, 2000);

		Sleep(2000);
		
		uint8_t progress = 1;
		while (progress) {

			// Query status
			action.action = ERASESTATUS;
			action.address = 0;
			rc = libusb_bulk_transfer(dev_handle, 1, (uint8_t*)&action, sizeof(action), &transferred, 1000);
			if (rc != 0) {
				printf("Send error: %d\r\n", rc);
				fclose(stream);
				return -105;
			}

			// Receive progress indicator
			rc = libusb_bulk_transfer(dev_handle, 0x82, (uint8_t*)&reply, sizeof(reply), &transferred, 2000);
			if (rc == 0) {
				if (reply.reserved >= 100) {
					reply.reserved = 100;
					progress = 0;
				}
				printf("Erasing: %d %% \r", reply.reserved);
			}
			else {
				printf("Erase error\r\n");
				progress = 0;
				fclose(stream);
				return -106;
			}

			Sleep(100);
		}
		printf("\r\n");

		float percent = 0.f;
		float fpercent = 100.f / (fileSize / 512);
		bytesLeft = fileSize;

		while (bytesLeft)
		{
			memset(readBuffer, 0xAA, sizeof(readBuffer));
#ifdef __GNUC__
			bytesRead = (uint32_t)fread(&readBuffer, 1, sizeof(readBuffer), stream);
#else
			bytesRead = (uint32_t)fread_s(&readBuffer, sizeof(readBuffer), 1, sizeof(readBuffer), stream);
#endif		
			bytesLeft -= bytesRead;

			CRC32 = crc32c(readBuffer, bytesRead, CRC32);
			
			if (0 != transferBlob(dev_handle, readBuffer, sizeof(readBuffer), flashAddress + offset))
			{
				break;
			}
			offset += IFLASH_PAGE_SIZE;

			percent += fpercent;
			if (percent > 100) percent = 100;
			printf("Flashing: %d %% \r", (uint32_t)percent);
		}
		CRC32 = ~CRC32;
		printf("\r\n");
		printf("Flashing signature\r\n");
		printf("File CRC: %08x\r\n", CRC32);

		memset(readBuffer, 0x00, sizeof(readBuffer));
		readBuffer[512 - 5] = (fileSize >> 24u) & 0xFFu;
		readBuffer[512 - 6] = (fileSize >> 16u) & 0xFFu;
		readBuffer[512 - 7] = (fileSize >> 8u) & 0xFFu;
		readBuffer[512 - 8] = (fileSize >> 0u) & 0xFFu;
		readBuffer[512 - 1] = (CRC32 >> 24u) & 0xFFu;
		readBuffer[512 - 2] = (CRC32 >> 16u) & 0xFFu;
		readBuffer[512 - 3] = (CRC32 >>  8u) & 0xFFu;
		readBuffer[512 - 4] = (CRC32 >>  0u) & 0xFFu;

		if (0 != transferBlob(dev_handle, readBuffer, sizeof(readBuffer), 0x005FFE00U))
		{
			printf("Failed flashing signature\r\n");
			fclose(stream);
			return -107;
		}
		else
		{
			action.action = RESET;
			libusb_bulk_transfer(dev_handle, 1, (uint8_t*)&action, sizeof(action), &transferred, 1000);
			printf("Signature written!\r\n");
		}

		fclose(stream);
	}
	else
	{
		printf("Failed to open %s", file);
		return -108;
	}

	return 0;
}
