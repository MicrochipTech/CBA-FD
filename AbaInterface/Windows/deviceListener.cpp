#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <strsafe.h>
#include "deviceListener.h"

#define WND_CLASS_NAME TEXT("MBAUSBListener")

void deviceListener::stopListen()
{
    if (m_hWnd != nullptr) {
        SendMessage(m_hWnd, WM_CLOSE, 0, 0);
        WaitForSingleObject(m_thread, INFINITE);
    }
}

bool deviceListener::startListen()
{
    bool bret = false;
    if (INVALID_HANDLE_VALUE == m_thread)
    {
        m_thread = CreateThread(0, 0, windowThreadWrapper, this, 0, 0);
        if (INVALID_HANDLE_VALUE != m_thread)
        {
            bret = true;
        }
    }
    return bret;
}

bool deviceListener::isFinished()
{
    bool bret = false;
    if (WAIT_OBJECT_0 == WaitForSingleObject(m_thread, INFINITE))
    {
        bret = true;
    }
    return bret;
}

deviceListener::deviceListener() :
    m_deviceIdentString(nullptr),
    m_hWnd(nullptr),
    m_notify(nullptr),
    m_thread(INVALID_HANDLE_VALUE),
    m_notifyCallback(nullptr)
{
    memset(&m_guid, 0, sizeof(m_guid));
}

deviceListener::~deviceListener()
{
    if (m_hWnd != nullptr)
    {
        stopListen();
    }
}

bool deviceListener::init()
{
    return InitWindowClass(GetModuleHandle(0));
}

bool deviceListener::addDevice(GUID guid, const wchar_t* deviceIdentString)
{
    if (nullptr == m_deviceIdentString)
    {
        size_t len = wcslen(deviceIdentString) * 2 + 2;
        auto tmpStr = new wchar_t[len];
        wcscpy_s(tmpStr, len, deviceIdentString);
        m_deviceIdentString = tmpStr;
        m_guid = guid;
        return true;
    }
    return false;
}

bool deviceListener::addListener(void* ptr)
{
    return false;
}

bool deviceListener::registerNotification()
{
    DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;

    ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
    NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
    NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    NotificationFilter.dbcc_classguid = m_guid;

    m_notify = RegisterDeviceNotification(
        m_hWnd,
        &NotificationFilter,
        DEVICE_NOTIFY_WINDOW_HANDLE | DEVICE_NOTIFY_ALL_INTERFACE_CLASSES
    );

    return (NULL == m_notify) ? FALSE : TRUE;
}

void deviceListener::deviceConnect(PDEV_BROADCAST_DEVICEINTERFACE dev)
{
    TCHAR strBuff[256] = { 0 };
    if (0 == wcsncmp(dev->dbcc_name, m_deviceIdentString, wcslen(m_deviceIdentString)))
    {
        if (dev->dbcc_classguid == m_guid)
        {
            int tokctr = 0;
            wchar_t* context = 0;
            wchar_t* token = wcstok_s(dev->dbcc_name, L"#", &context);

            while (token)
            {
                if (tokctr == 2)
                {
                    mba_serial_t tmpSerial;
                    size_t szCharsConverted = 0;
                    wcstombs_s(&szCharsConverted, tmpSerial.octet, sizeof(tmpSerial.octet), token, _TRUNCATE);
                    hotplug_windows(tmpSerial, aba::usb::CONNECT);
                }
                token = wcstok_s(nullptr, L"#", &context);
                tokctr++;
            }
        }
    }
}

void deviceListener::deviceDisconnect(PDEV_BROADCAST_DEVICEINTERFACE dev)
{
    TCHAR strBuff[256] = { 0 };

    if (0 == wcsncmp(dev->dbcc_name, m_deviceIdentString, wcslen(m_deviceIdentString)))
    {
        if (dev->dbcc_classguid == m_guid)
        {
            int tokctr = 0;
            wchar_t* context = 0;
            wchar_t* token = wcstok_s(dev->dbcc_name, L"#", &context);

            while (token)
            {
                if (tokctr == 2)
                {
                    mba_serial_t tmpSerial;
                    size_t szCharsConverted = 0;
                    wcstombs_s(&szCharsConverted, tmpSerial.octet, sizeof(tmpSerial.octet), token, _TRUNCATE);
                    hotplug_windows(tmpSerial, aba::usb::DISCONNECT);
                }
                token = wcstok_s(nullptr, L"#", &context);
                tokctr++;
            }
        }
    }
}

INT_PTR WINAPI deviceListener::WinProcCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    LRESULT lRet = 1;
    static deviceListener* dev = nullptr;

    switch (message)
    {
    case WM_NCCREATE:
    {
        LPCREATESTRUCTA cs = (LPCREATESTRUCTA)lParam;
        dev = reinterpret_cast<deviceListener*>(cs->lpCreateParams);
        break;
    }

    case WM_CREATE:
        dev->m_hWnd = hWnd;
        dev->registerNotification(); 
        break;

    case WM_DEVICECHANGE:
    {
        PDEV_BROADCAST_DEVICEINTERFACE b = reinterpret_cast<PDEV_BROADCAST_DEVICEINTERFACE>(lParam);

        switch (wParam)
        {
        case DBT_DEVICEARRIVAL:
            dev->deviceConnect(b);
            break;
        
        case DBT_DEVICEREMOVECOMPLETE:
            dev->deviceDisconnect(b);
            break;
        }
    }
    break;

    case WM_CLOSE:
        DestroyWindow(hWnd);
        break;

    case WM_DESTROY:
        UnregisterDeviceNotification(dev->m_notify);
        PostQuitMessage(0);
        break;

    default:
        // Send all other messages on to the default windows handler.
        lRet = DefWindowProc(hWnd, message, wParam, lParam);
        break;
    }
    
    return lRet;
}

void deviceListener::messagePump()
{
    MSG msg;
    int retVal;

    while((retVal = GetMessage(&msg, NULL, 0, 0)) != 0)
    {
        if(retVal == -1)
        {
            break;
        }
        else
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }
}

bool deviceListener::InitWindowClass(HINSTANCE hInst)
{
    WNDCLASSEX wndClass;

    wndClass.cbSize = sizeof(WNDCLASSEX);
    wndClass.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    wndClass.hInstance = hInst;
    wndClass.lpfnWndProc = reinterpret_cast<WNDPROC>(WinProcCallback);
    wndClass.cbClsExtra = 0;
    wndClass.cbWndExtra = 0;
    wndClass.hIcon = LoadIcon(0, IDI_APPLICATION);
    wndClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW);
    wndClass.hCursor = LoadCursor(0, IDC_ARROW);
    wndClass.lpszClassName = WND_CLASS_NAME;
    wndClass.lpszMenuName = NULL;
    wndClass.hIconSm = wndClass.hIcon;

    if (!RegisterClassEx(&wndClass))
    {
        return FALSE;
    }
    return TRUE;
}

DWORD WINAPI deviceListener::windowThreadWrapper(LPVOID lpThreadParameter)
{
    reinterpret_cast<deviceListener*>(lpThreadParameter)->windowThread();
    return 0;
}

void deviceListener::windowThread()
{
    HWND hWnd = CreateWindowEx(
        WS_EX_CLIENTEDGE | WS_EX_APPWINDOW,
        WND_CLASS_NAME,
        L"",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, 0,
        640, 480,
        NULL, NULL,
        GetModuleHandle(0),
        this);

    if (hWnd == NULL)
    {
        return;
    }

    messagePump();
}
