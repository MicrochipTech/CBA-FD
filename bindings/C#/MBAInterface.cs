using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace MBA
{
    public class MBAInterface
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void mbaCallback (IntPtr message, IntPtr userData);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void unload();

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 enumDevices(ref IntPtr deviceList);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 openDevice(ref IntPtr abaDevice, Byte[] serial);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 closeDevice(IntPtr abaDevice);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 registerCallback([MarshalAs(UnmanagedType.FunctionPtr)] mbaCallback cb, IntPtr userData);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 unregisterCallback();

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 CAN_SendFrame(IntPtr aba_device, Int32 busId, UInt32 canId, Byte[] payload, Byte dlc, Byte flags, UInt16 timeout);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 CAN_SetSpeed(IntPtr aba_device, Int32 busId, UInt32 canSpeed, UInt32 canFdSpeed);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 CAN_SetMode(IntPtr aba_device, Int32 busId, Byte canMode, Byte testMode, Byte autoRetryEnabled);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 CAN_Reset(IntPtr aba_device, Int32 busId);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 MBA_Reset(IntPtr aba_device, Int32 resetMode);

        [DllImport("MbaInterface.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 MBA_GetVersion(IntPtr aba_device);

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct Device
        {
            public UInt32 DeviceType;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 40)]
            public byte[] Serial;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CanFrame
        {
            public UInt32 BusId;
            public UInt32 CanId;
            public UInt64 Timestamp;
            public Byte DLC;
            public Byte Flags;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
            public Byte[] Data;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CanError
        {
            public UInt32 BusId;
            public Byte BusOff;
            public Byte TEC;
            public Byte REC;
            public Byte LEC;
            public Byte DLEC;
            public UInt64 Timestamp;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct MbaMessage
        {
            public Byte Instance;
            public Byte MessageType;
            public Byte Command;
            public Byte Reserved;
            public IntPtr Data;
            public UInt32 DataLength;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct MbaVersion
        {
            public UInt32 Firmware;
            public UInt32 API;
            public UInt32 CAN;
            public UInt32 Bootloader;
            public UInt32 Hardware;
            public UInt32 Hash;
        }
    }

    public enum CANFRAME_FLAG : Byte
    {
        CANFRAME_FLAG_NONE = 0x0,
        CANFRAME_FLAG_EXTENDED = 0x1,
        CANFRAME_FLAG_FD = 0x2,
        CANFRAME_FLAG_BRS = 0x4
    }

    public enum CAN_MODE : Byte
    {
        CAN_MODE_CLASSIC = 0,
        CAN_MODE_FDNISO,
        CAN_MODE_FDISO,
    }

    public enum CAN_TESTMODE : Byte
    {
        CAN_TESTMODE_INIT = 0,
        CAN_TESTMODE_NORMAL,
        CAN_TESTMODE_LISTENONLY,
        CAN_TESTMODE_LOOPBACK,
        CAN_TESTMODE_EXTLOOPBACK,
    };

    public enum CAN_BUS : Int32
    {
        CAN_BUS_0 = 0,
        CAN_BUS_1,
        CAN_BUS_MAX
    };

    public enum CAN_MSG : Byte
    {
        CAN_MSG_RX = 0,
        CAN_MSG_TX,
        CAN_MSG_ERR,
        CAN_MSG_DBG,
        CAN_MSG_BITRATE,
        CAN_MSG_MODE
    }

    public enum MBA_CMD: Byte
    {
        MBA_CORE = 0,
        MBA_CAN,
        MBA_CXPI
    };

    public enum MBA_MSG: Byte
    {
        MBA_RESERVED = 0,
        MBA_TIMESTAMP,
        MBA_VERSION,
        MBA_USBERROR,
        MBA_DEVICEDISCONNECTED,
        MBA_DEVICERECONNECTED
    }

    public enum MBA_RESET: Int32
    {
        MBA_RESET_APPLICATION,
        MBA_RESET_BOOTLOADER
    };

    public enum MBA_ERR : Int32
    {
        E_OK = 0,       /// All okay
        E_ERR = -1,     /// Error
        E_INIT = -2,    /// Device not initialized
        E_ARG = -3,     /// Wrong argument
        E_DEV = -4,     /// Called function on wrong device
        E_NODEV = -5,   /// No device available
        E_MEM = -6,     /// Memory allocation error
        E_DISC = -7     /// Device has been disconnected (without being closed before)
    }
}
