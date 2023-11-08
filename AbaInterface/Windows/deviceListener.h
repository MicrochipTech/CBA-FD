#pragma once

#include <Windows.h>
#include <dbt.h>

#include <deviceListenerCb.h>

// #define DBGSCRN

#define DEVICE_ARRIVED 1
#define DEVICE_LEFT 2

class deviceListener
{

public:

    deviceListener();
    virtual ~deviceListener();
    bool init();
    bool addDevice(GUID guid, const wchar_t* deviceIdentString);
    bool addListener(void* ptr);
    bool startListen();
    void stopListen();
    bool isFinished();

    void registerCallback(notificationCallback* callback);
    
private:

    void windowThread();
    static DWORD WINAPI windowThreadWrapper(LPVOID lpThreadParameter);

    static INT_PTR WINAPI WinProcCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
    //static INT_PTR WINAPI WinProcWrapper(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    void messagePump();
    bool registerNotification();
    bool InitWindowClass(HINSTANCE hInst);


    void deviceConnect(PDEV_BROADCAST_DEVICEINTERFACE dev);
    void deviceDisconnect(PDEV_BROADCAST_DEVICEINTERFACE dev);

    wchar_t* m_deviceIdentString;

    GUID m_guid;
    HWND m_hWnd;
    HDEVNOTIFY m_notify;
    HANDLE m_thread;
    
    notificationCallback* m_notifyCallback;
};

