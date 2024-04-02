#include <iostream>
#include <string>
#include <assert.h>
#include <stdint.h>
#include <CLI.hpp>
#include <MbaInterface.h>
#include "pcap.hpp"
#include "pipe.hpp"

#define LINKTYPE_CAN_SOCKETCAN 227u

std::string version = "MBA Extcap Version: 1.0.1";

sPipe pipe = {};
volatile uint32_t terminateLoop = 0;
volatile uint32_t enableProcessing = 0;

void handle_reply(const mba_message_t* message, void* user_data)
{
    if (enableProcessing < 1u)
    {
        return;
    }
 
    uint32_t frameSize;
    uint32_t rc = 0;

    switch (message->messageType)
    {
    case MBA_CAN:
        if (message->command == CAN_MSG_RX)
        {
            can_frame_t frame = {};
            auto frame_in = reinterpret_cast<const CanFrame*>(message->data);
            
            frame.can_id = frame_in->id;
            frame.dlc = frame_in->dlc;
            memcpy_s(frame.data, sizeof(frame.data), frame_in->data, frame_in->dlc);
            frame.flags = frame_in->flags & CANFRAME_FLAG_FD ? 1 : 0;

            if (frame_in->flags & CANFRAME_FLAG_EXTENDED) {
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
            can_frame_t frame = {};
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
            if (rc == 0) {
                terminateLoop = 1;
            }
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
        std::cout << "interface {value=CAN_"
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
        int32_t type = 0;
        mba_serial_t serial = {};
        mba_handle_t* mba_device = nullptr;

        size_t sp = set.iface.find('_');
        if (sp != std::string::npos)
        {
            std::string prefix = set.iface.substr(0, sp);
            std::string interface = set.iface.substr(sp + 1u);
            strcpy_s(reinterpret_cast<char*>(serial.octet), sizeof(serial.octet), interface.c_str());

            if (prefix == "CAN") {
                type = 1u;
            }
        }

        if (type == 0) {
            return -1;
        }

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

            if (set.ack == "true") {
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

    int32_t ret = -1;
    settings_t settings = {};
    CLI::App extcap{ "Microchip Bus Analyzer" };
    auto vers = extcap.add_flag("-v,--version");
    auto dlts = extcap.add_flag("--extcap-dlts");
    auto intf = extcap.add_flag("--extcap-interfaces");
    extcap.add_flag("--extcap-version");

    //
    auto conf = extcap.add_subcommand("extcap-config");
    conf->alias("--extcap-config");
    conf->add_option("--extcap-interface", settings.iface)->required();

    //
    auto* capt = extcap.add_subcommand("capture");
    capt->alias("--capture");
    capt->add_option("--extcap-interface", settings.iface)->required();
    capt->add_option("--nspeed0", settings.nspeed0)->default_val(500);
    capt->add_option("--dspeed0", settings.dspeed0)->default_val(2000);
    capt->add_option("--nspeed1", settings.nspeed1)->default_val(500);
    capt->add_option("--dspeed1", settings.dspeed1)->default_val(2000);
    capt->add_option("--fifo", settings.pipe)->required();
    capt->add_option("--ack", settings.ack)->default_str("false");   

    CLI11_PARSE(extcap, argc, argv);
    if (intf->count()) {
        // Wireshark requires returncode 0 on success
        ret = printInterfaces() ? 0 : 1;
    }
    else if (vers->count())
    {
        std::cout << version << std::endl;
        ret = 0;
    }
    else if (dlts->count()) {
        ret = showDlts(settings);
    }
    else if(capt->count()) {
        ret = startCapture(settings);
    }
    else if (conf->count()) {
        ret = showConfig(settings);
    }

    return ret;
}
