#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <string.h>

#include "compiler_w.h"
#include "libusb.h"
#include "Bldr_Protocol.h"
#include "Bootloader_flash.h"

static void print_devs(libusb_device **devs)
{
	libusb_device *dev;
	int i = 0, j = 0;
	uint8_t path[8];

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
			return;
		}

		printf("%04x:%04x (bus %d, device %d)",
			desc.idVendor, desc.idProduct,
			libusb_get_bus_number(dev), libusb_get_device_address(dev));

		r = libusb_get_port_numbers(dev, path, sizeof(path));
		if (r > 0) {
			printf(" path: %d", path[0]);
			for (j = 1; j < r; j++)
				printf(".%d", path[j]);
		}
		printf("\n");
	}
}

int getDevice(libusb_device** devs, uint16_t vendorId, uint16_t productId)
{
	libusb_device* dev = nullptr;
	uint32_t i = 0;

	while (devs[i] != NULL) {
		struct libusb_device_descriptor desc;
		int rc = libusb_get_device_descriptor(devs[i], &desc);
		if (rc < 0) {
			break;
		}

		if (desc.idVendor == 0x04D8u && desc.idProduct == productId) {
			return i;
		}

		i++;
	}
	
	return -1;
}

void print_configuration(libusb_device_handle* hDevice, struct libusb_config_descriptor *config)
{
	printf("\nInterface Descriptors: ");
	printf("\n\tNumber of Interfaces: %d", config->bNumInterfaces);
	printf("\n\tLength: %d", config->bLength);
	printf("\n\tDesc_Type: %d", config->bDescriptorType);
	printf("\n\tConfig_index: %d", config->iConfiguration);
	printf("\n\tTotal length: %d", config->wTotalLength);
	printf("\n\tConfiguration Value: %d", config->bConfigurationValue);
	printf("\n\tConfiguration Attributes: %d", config->bmAttributes);
	printf("\n\tMaxPower(mA): %d\n", config->MaxPower);
}

int alt_interface, interface_number;

const struct libusb_endpoint_descriptor * active_config(struct libusb_device *dev, struct libusb_device_handle *handle)
{
	struct libusb_device_handle *hDevice_req;
	struct libusb_config_descriptor *config;
	const struct libusb_endpoint_descriptor *endpoint = 0;
	int altsetting_index, interface_index = 0, ret_active;

	hDevice_req = handle;

	ret_active = libusb_get_active_config_descriptor(dev, &config);
	print_configuration(hDevice_req, config);

	for (interface_index = 0; interface_index<config->bNumInterfaces; interface_index++)
	{
		const struct libusb_interface *iface = &config->interface[interface_index];
		for (altsetting_index = 0; altsetting_index<iface->num_altsetting; altsetting_index++)
		{
			const struct libusb_interface_descriptor *altsetting = &iface->altsetting[altsetting_index];

			int endpoint_index;
			for (endpoint_index = 0; endpoint_index<altsetting->bNumEndpoints; endpoint_index++)
			{
				const struct libusb_endpoint_descriptor *ep = &altsetting->endpoint[endpoint_index];
				endpoint = ep;
				alt_interface = altsetting->bAlternateSetting;
				interface_number = altsetting->bInterfaceNumber;

				printf("\nEndPoint Descriptors: ");
				printf("\n\tSize of EndPoint Descriptor: %d", endpoint->bLength);
				printf("\n\tType of Descriptor: %d", endpoint->bDescriptorType);
				printf("\n\tEndpoint Address: 0x0%x", endpoint->bEndpointAddress);
				printf("\n\tMaximum Packet Size: %x", endpoint->wMaxPacketSize);
				printf("\n\tAttributes applied to Endpoint: %d", endpoint->bmAttributes);
				printf("\n\tInterval for Polling for data Tranfer: %d\n", endpoint->bInterval);
			}
		}
	}
	libusb_free_config_descriptor(NULL);
	return endpoint;
}



int main(int argc, char** argv)
{
	printf("Starting main\r\n");

	if (argc != 2) {
		printf("Firmware missing!\r\n%s <firmware.bin>\r\n", argv[0]);
		return -1;
	}

	int rc = libusb_init(nullptr);
	if (rc < 0) {
		printf("Failed to getDevice %d", rc);
		return rc;
	}

	libusb_device** devs;
	auto devCnt = libusb_get_device_list(nullptr, &devs);

	print_devs(devs);

	// Not present in header file yet
	// libusb_set_option(nullptr, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);

	auto devId = getDevice(devs, 0, 0x0AB0);

	if (devId < 0) {
		printf("Failed to getDevice %d", devId);
		return devId;
	}

	libusb_device_handle* dev_handle = 0;
	rc = libusb_open(devs[devId], &dev_handle);

	if (rc < 0) {
		printf("Failed to open lib usb %d", rc);
		return rc;
	}

	uint8_t dataBuffer[1024];
	auto len = sizeof(dataBuffer);
	memset(dataBuffer, 0xAA, len);

	int transferred = 0;

	libusb_config_descriptor* config = 0;
	auto ret_active = libusb_get_active_config_descriptor(devs[devId], &config);
	print_configuration(dev_handle, config);
	active_config(devs[devId], dev_handle);


	for(int i=0; i<1; i++)
	{
		rc = libusb_claim_interface(dev_handle, i);
		if (rc != 0)
		{
			printf("Error while claiming endpoint %d\r\n", i);
			return -10;
		}
	}

	libusb_clear_halt(dev_handle, 0);

	auto size = libusb_get_max_packet_size(devs[devId], 1);
	printf("Max size: %d\r\n", size);

	size = libusb_get_max_packet_size(devs[devId], 0x82);
	printf("Max size: %d\r\n", size);

	rc = sendFirmware(argv[1], 0x00420000U, dev_handle);
	if (rc == 0)
	{
		printf("Success\r\n");
	}
	else
	{
		printf("Error while sending fimrware %d\r\n", rc);
	}

	return rc;
}