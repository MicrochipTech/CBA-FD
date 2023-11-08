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

/*////////////////////////////////////////////////////////////////////////////////

This file contains the Windows specific stubs

////////////////////////////////////////////////////////////////////////////////*/

#include <Windows.h>
#include "MbaInterface.h"
#include "deviceListener.h"

HINSTANCE hDll = nullptr;
deviceListener* devList = nullptr;

void setupDeviceListener(HINSTANCE hInst)
{
    if (nullptr == devList)
    {
        //+     dbcc_classguid  {A5DCBF10-6530-11D2-901F-00C04FB951ED}  _GUID
        //+     dbcc_classguid	{9F543223-CEDE-4FA3-B376-A25CE9A30E74}  _GUID
        //+     dbcc_classguid	{DEE824EF-729B-4A0E-9C14-B7117D33A817}  _GUID
        GUID devGUID = { 0x9F543223u, 0xCEDEu, 0x4FA3u, 0xB3u, 0x76u, 0xA2u, 0x5Cu, 0xE9u, 0xA3u, 0x0Eu, 0x74u };
        const wchar_t* deviceIdentString = L"\\\\?\\USB#VID_04D8&PID_0AB";

        devList = new deviceListener();
        devList->init();
        devList->addDevice(devGUID, deviceIdentString);
        devList->startListen();
    }
}

int LibraryInit()
{
    setupDeviceListener(hDll);
    return 0;
}

int LibraryUnload()
{
    if (nullptr != devList) {
        delete devList;
        devList = nullptr;
    }
    return 0;
}

BOOL WINAPI DllMain(
	HINSTANCE hinstDLL,
	DWORD     fdwReason,
	LPVOID    lpvReserved
)
{
	switch (fdwReason)
	{
	case DLL_PROCESS_ATTACH:
        hDll = hinstDLL;
		break;
	case DLL_PROCESS_DETACH:
        // When the library has exited gracefully, unload() does nothing.
        // On Windows10 threads are already terminated when getting here, so it's safe to remove it anyway?
        unload();
		break;
	case DLL_THREAD_ATTACH:
		break;
	case DLL_THREAD_DETACH:
		break;
	}

	return TRUE;
}