using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using static MBA.MBAInterface;

namespace MBA
{
    public class MBADevice
    {
        IntPtr m_device;
        GCHandle m_handle;
        MBA_Callback m_userCallback;
        MBAInterface.mbaCallback m_callback;
        public delegate void MBA_Callback(MBAInterface.MbaMessage msg);

        private MBADevice()
        {
            // Can only be instantiated by calling Create(...)
            m_device = IntPtr.Zero;
            m_handle = GCHandle.Alloc(this);
            m_userCallback = null;
        }

        private static T[] PtrToStructureArray<T>(IntPtr ptr, int count) where T : struct
        {
            T[] array = new T[count];
            int structSize = Marshal.SizeOf(typeof(T));

            for (int i=0; i<count; i++)
            {
                IntPtr currentPtr = new IntPtr(ptr.ToInt64() + (structSize * i));
                array[i] = Marshal.PtrToStructure<T>(currentPtr);
            }

            return array;
        }

        public static Int32 Enumerate(out List<MBAInterface.Device> devices)
        {
            devices = new List<MBAInterface.Device>();
            IntPtr pDevices = new IntPtr();
            Int32 devCount = MBAInterface.enumDevices(ref pDevices);

            var devs = PtrToStructureArray<MBAInterface.Device>(pDevices, devCount);
            foreach(MBAInterface.Device dev in devs)
            {
                devices.Add(dev);
            }

            return devices.Count;
        }

        public static MBADevice Create(String serial, MBA_Callback callback)
        {
            MBADevice dev = new MBADevice();
            var ret = MBAInterface.openDevice(ref dev.m_device, Encoding.ASCII.GetBytes(serial));
            if(ret == (int)MBA_ERR.E_OK)
            {
                dev.m_callback = new MBAInterface.mbaCallback(MBADevice.Callback);
                dev.m_userCallback = callback;
                MBAInterface.registerCallback(dev.m_callback, GCHandle.ToIntPtr(dev.m_handle));
            }
            else
            {
                dev = null;
            }
            
            return dev;
        }

        ~MBADevice()
        {
            // unregisterCallback is always valid
            MBAInterface.unregisterCallback();
            if (m_device != IntPtr.Zero)
            {
                // only close device handles that were opened
                MBAInterface.closeDevice(m_device);
                m_device = IntPtr.Zero;
            }
            m_handle.Free();
        }

        static void Callback(IntPtr message, IntPtr user_data)
        {
            if (user_data != IntPtr.Zero)
            {
                var func = (MBADevice)GCHandle.FromIntPtr(user_data).Target;
                func.m_userCallback(Marshal.PtrToStructure<MBAInterface.MbaMessage>(message));
            }
            else
            {
                throw new Exception("FATAL: No user data provided by callback!");
            }
        }

        public Int32 CAN_SendFrame(CAN_BUS busId, UInt32 canId, Byte[] payload, Byte dlc, Byte flags, UInt16 timeout)
        {
            return MBAInterface.CAN_SendFrame(m_device, (Int32)busId, canId, payload, dlc, flags, timeout);
        }

        public Int32 CAN_SetSpeed(CAN_BUS busId, UInt32 canSpeed, UInt32 canFdSpeed)
        {
            return MBAInterface.CAN_SetSpeed(m_device, (Int32)busId, canSpeed, canFdSpeed);
        }

        public Int32 CAN_SetMode(CAN_BUS busId, CAN_MODE canMode, CAN_TESTMODE testMode, Byte autoRetryEnabled)
        {
            return MBAInterface.CAN_SetMode(m_device, (Int32)busId, (Byte)canMode, (Byte)testMode, autoRetryEnabled);
        }

        public Int32 CAN_Reset(CAN_BUS busId)
        {
            return MBAInterface.CAN_Reset(m_device, (Int32)busId);
        }

        public Int32 MBA_Reset(MBA_RESET resetMode)
        {
            return MBAInterface.MBA_Reset(m_device, (Int32)resetMode);
        }

        public Int32 MBA_GetVersion()
        {
            return MBAInterface.MBA_GetVersion(m_device);
        }

    }
}
