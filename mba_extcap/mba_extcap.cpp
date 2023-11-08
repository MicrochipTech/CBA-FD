#include <iostream>
#include <string>
#include <assert.h>
#include <stdint.h>
#include <CLI/CLI.hpp>
#include <MbaInterface.h>
#include "pcap.hpp"
#include "pipe.hpp"

#include <windows.h>

#define LINKTYPE_CAN_SOCKETCAN 227

sPipe pipe = {};
volatile uint32_t terminateLoop = 0;
volatile uint32_t enableProcessing = 0;

struct can_frame_t
{
	uint32_t can_id;
	uint8_t dlc;
	uint8_t flags;
	uint8_t res[2];
	uint8_t data[64];
};

enum _t_error_
{
    TE_NOERR,
    TE_STUFFERR,
    TE_FORMERR,
    TE_ACKERR,
    TE_BIT1ERR,
    TE_BIT0ERR,
    TE_CRCERR,
    TE_NOCHANGE
};

void handle_reply(const mba_message_t* message, void* user_data)
{
    if (enableProcessing < 1)
    {
        return;
    }
 
    uint32_t frameSize;
    uint32_t rc = 0;
    can_frame_t frame = {};

	switch (message->messageType)
	{
	case MBA_CAN:
		if (message->command == CAN_MSG_RX)
		{
            auto frame_in = reinterpret_cast<const CanFrame*>(message->data);
            
			frame.can_id = frame_in->id;
			frame.dlc = frame_in->dlc;
			memcpy_s(frame.data, sizeof(frame.data), frame_in->data, frame_in->dlc);
			frame.flags = frame_in->flags & CANFRAME_FLAG_FD ? 1 : 0;

            if (frame_in->flags & CANFRAME_FLAG_EXTENDED)
            {
                frame.can_id |= CAN_EFF_FLAG; // Set extended
            }
            frame.can_id = _byteswap_ulong(frame.can_id);

            frameSize = 8u + frame.dlc;
            rc = pcapng_write_enhanced_packet(pipe, frame_in->busId, frame_in->timestamp >> 32, frame_in->timestamp & 0xFFFFFFFF, frameSize, reinterpret_cast<const uint8_t*>(&frame));
            if (rc == 0)
                terminateLoop = 1;
        }
		else if (message->command == CAN_MSG_ERR)
        {
            auto frame_in = reinterpret_cast<const CanError*>(message->data);

            frame.can_id = CAN_ERR_FLAG; // Set error frame
            
            if (frame_in->busOff) {
                frame.can_id |= CAN_ERR_BUSOFF;
            }

            if (frame_in->rec > 96) {
                frame.data[2] |= CAN_ERR_CRTL_RX_WARNING;
            }
            else if (frame_in->rec > 127) {
                frame.data[2] |= CAN_ERR_CRTL_RX_PASSIVE;
            }

            switch (frame_in->lec)
            {
            case TE_STUFFERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_STUFF;
                break;
            case TE_FORMERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_FORM;
                break;
            case TE_ACKERR:
                frame.can_id |= CAN_ERR_ACK;
                break;
            case TE_BIT1ERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_BIT1;
                break;
            case TE_BIT0ERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_BIT0;
                break;
            case TE_CRCERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_UNSPEC;
                break;
            default:
            case TE_NOERR:
            case TE_NOCHANGE:
                break;
            }

            switch (frame_in->dlec)
            {
            case TE_STUFFERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_STUFF;
                break;
            case TE_FORMERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_FORM;
                break;
            case TE_ACKERR:
                frame.can_id |= CAN_ERR_ACK;
                break;
            case TE_BIT1ERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_BIT1;
                break;
            case TE_BIT0ERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_BIT0;
                break;
            case TE_CRCERR:
                frame.can_id |= CAN_ERR_PROT;
                frame.data[2] = CAN_ERR_PROT_UNSPEC;
                break;
            default:
            case TE_NOERR:
            case TE_NOCHANGE:
                break;
            }

            frame.data[5] = frame_in->lec;
            frame.data[6] = frame_in->dlec;
            frame.data[7] = frame_in->rec;
            frame.can_id = _byteswap_ulong(frame.can_id);
            frame.dlc = 8u;

            frameSize = 8u + frame.dlc;
            rc = pcapng_write_enhanced_packet(pipe, frame_in->busId, frame_in->timestamp >> 32, frame_in->timestamp & 0xFFFFFFFF, frameSize, reinterpret_cast<const uint8_t*>(&frame));
            if (rc == 0)
                terminateLoop = 1;
        }

		break;
	case MBA_CORE:
        terminateLoop = 1;
        break;
	}
}

static int32_t printInterfaces()
{
    mba_device_t* device_list = nullptr;
	int32_t numDevices = enumDevices(&device_list);

	std::cout << "extcap {version=1.0}" << std::endl;

	for (int32_t i = 0; i < numDevices; i++)
	{
		std::cout << "interface {value="
			<< reinterpret_cast<char*>(device_list[i].serial.octet)
			<< "}{display=Microchip CAN Bus Analyzer ("
			<< reinterpret_cast<char*>(device_list[i].serial.octet)
			<< ")}" << std::endl;
	}

	return numDevices;
}

static int32_t startCapture(const settings_t& set)
{
    bool rc = open_pipe(set, pipe);

    if (rc)
    {
        mba_serial_t serial = {};
        mba_handle_t* mba_device = nullptr;
        strcpy_s((char*)serial.octet, sizeof(serial.octet), set.iface.c_str());

        registerCallback(handle_reply, 0);
        if (E_OK == openDevice(&mba_device, &serial))
        {
            CAN_Reset(mba_device, CAN_BUS_0);
            CAN_Reset(mba_device, CAN_BUS_1);

            CAN_SetSpeed(mba_device, CAN_BUS_0, set.nspeed0, set.dspeed0);
            CAN_SetSpeed(mba_device, CAN_BUS_1, set.nspeed1, set.dspeed1);

            pcapng_write_section_header(pipe);
            pcapng_write_interface_desc_options(pipe, LINKTYPE_CAN_SOCKETCAN, 128, "Interface 0", "CAN0");
            pcapng_write_interface_desc_options(pipe, LINKTYPE_CAN_SOCKETCAN, 128, "Interface 1", "CAN1");

            enableProcessing = 1;

            if (set.ack) {
                CAN_SetMode(mba_device, CAN_BUS_0, CAN_MODE_FDISO, CAN_TESTMODE_NORMAL, 0);
                CAN_SetMode(mba_device, CAN_BUS_1, CAN_MODE_FDISO, CAN_TESTMODE_NORMAL, 0);
            } else {
                CAN_SetMode(mba_device, CAN_BUS_0, CAN_MODE_FDISO, CAN_TESTMODE_LISTENONLY, 0);
                CAN_SetMode(mba_device, CAN_BUS_1, CAN_MODE_FDISO, CAN_TESTMODE_LISTENONLY, 0);
            }

            while (!terminateLoop) Sleep(100);

            closeDevice(mba_device);
        }
    }

	return 0;
}

static int32_t showConfig(const settings_t& set)
{
	std::cout << "arg {number=0}{call=--nspeed0}{display=CAN0: nominal speed}{type=selector}" << std::endl
		<< "value {arg=0}{value=100}{display=100kBit/s}{default=false}"     << std::endl
		<< "value {arg=0}{value=125}{display=125kBit/s}{default=false}"     << std::endl
		<< "value {arg=0}{value=250}{display=250kBit/s}{default=false}"     << std::endl
		<< "value {arg=0}{value=500}{display=500kBit/s}{default=true}"      << std::endl
		<< "value {arg=0}{value=1000}{display=1000kBit/s}{default=false}"   << std::endl;

	std::cout
		<< "arg {number=1}{call=--dspeed0}{display=CAN0: data speed}{type=selector}" << std::endl
		<< "value {arg=1}{value=1000}{display=1000kBit/s}{default=false}"   << std::endl
		<< "value {arg=1}{value=2000}{display=2000kBit/s}{default=true}"    << std::endl
		<< "value {arg=1}{value=3077}{display=3077kBit/s}{default=false}"   << std::endl
		<< "value {arg=1}{value=4000}{display=4000kBit/s}{default=false}"   << std::endl
		<< "value {arg=1}{value=5000}{display=5000kBit/s}{default=false}"   << std::endl
		<< "value {arg=1}{value=6667}{display=6667kBit/s}{default=false}"   << std::endl
		<< "value {arg=1}{value=8000}{display=8000kBit/s}{default=false}"   << std::endl;

	std::cout << "arg {number=2}{call=--nspeed1}{display=CAN1: nominal speed}{type=selector}" << std::endl
		<< "value {arg=2}{value=100}{display=100kBit/s}{default=false}"     << std::endl
		<< "value {arg=2}{value=125}{display=125kBit/s}{default=false}"     << std::endl
		<< "value {arg=2}{value=250}{display=250kBit/s}{default=false}"     << std::endl
		<< "value {arg=2}{value=500}{display=500kBit/s}{default=true}"      << std::endl
		<< "value {arg=2}{value=1000}{display=1000kBit/s}{default=false}"   << std::endl;

	std::cout << "arg {number=3}{call=--dspeed1}{display=CAN1: data speed}{type=selector}" << std::endl
		<< "value {arg=3}{value=1000}{display=1000kBit/s}{default=false}"   << std::endl
		<< "value {arg=3}{value=2000}{display=2000kBit/s}{default=true}"    << std::endl
		<< "value {arg=3}{value=3077}{display=3077kBit/s}{default=false}"   << std::endl
		<< "value {arg=3}{value=4000}{display=4000kBit/s}{default=false}"   << std::endl
		<< "value {arg=3}{value=5000}{display=5000kBit/s}{default=false}"   << std::endl
		<< "value {arg=3}{value=6667}{display=6667kBit/s}{default=false}"   << std::endl
		<< "value {arg=3}{value=8000}{display=8000kBit/s}{default=false}"   << std::endl;

    std::cout << "arg {number=4}{call=--ack}{display=Send acknowledgments}{type=boolean}" << std::endl;

	return 0;
}

static int32_t showDlts(const settings_t& set)
{
	std::cout << "dlt {number=227}{name=Microchip CAN Analyzer}{display=CAN/CAN-FD}" << std::endl;
	return 0;
}

int main(int argc, char* argv[])
{
    for (int i = 0; i < argc; i++)
        OutputDebugStringA(argv[i]);

    if ((argc == 2 || argc == 3) && _strcmpi(argv[1], "--extcap-interfaces") == 0) {
		printInterfaces();
		return 0;
	}

	int32_t ret = -1;
	int32_t iDlts = 0;
	int32_t iConfig = 0;
	int32_t iCapture = 0;
	std::string strInterface = "";

    settings_t settings = {};
	CLI::App extcap{ "Microchip Bus Analyzer" };
	extcap.add_option("--extcap-interface", settings.iface, "")->required();
	extcap.add_flag("--extcap-dlts", iDlts);
	extcap.add_flag("--extcap-config", iConfig);
	extcap.add_option("--fifo", settings.pipe)->required();

	auto ec_capture = extcap.add_flag("--capture", iCapture);
	ec_capture->needs(extcap.add_option("--nspeed0", settings.nspeed0)->default_val("500"));
	ec_capture->needs(extcap.add_option("--dspeed0", settings.dspeed0)->default_val("2000"));
	ec_capture->needs(extcap.add_option("--nspeed1", settings.nspeed1)->default_val("500"));
	ec_capture->needs(extcap.add_option("--dspeed1", settings.dspeed1)->default_val("2000"));
    ec_capture->needs(extcap.add_option("--ack", settings.ack)->default_val("0"));

	CLI11_PARSE(extcap, argc, argv);

	if (iCapture) {
		ret = startCapture(settings);
	}
	else if (iConfig) {
		ret = showConfig(settings);
	}
	else if (iDlts) {
		ret = showDlts(settings);
	}

	return ret;
}
