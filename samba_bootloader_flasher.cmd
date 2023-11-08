@rem Flash bootloader after a SAMV71 hard reset to SAM-BA mode.
@rem Bootloader sets TCM GPNVM[8:7] bits at startup.
@rem Download/unzip the SAM-BA tools from https://www.microchip.com/developmenttools/ProductDetails/PartNO/SAM-BA%20In-system%20Programmer
@rem This cmd file must be run from the artifacts directory
@rem Arg 1: COM port (e.g. COM1). In Windows Device Manager look for 'AT91 USB to Serial Converter' under Ports.
@rem Arg 2: app. If present will also flash the application.

@echo off

Setlocal

@echo Set boot mode to flash.
sam-ba_3.3.1\sam-ba.exe -p serial:%1 -d samv71 -a bootconfig -c writecfg:bootmode:flash
if %ERRORLEVEL% NEQ 0 GOTO ERROR

echo.
@echo Flash Bootloader.bin.
sam-ba_3.3.1\sam-ba.exe -p serial:%1 -d samv71 -a internalflash -c write:Bootloader.bin
if %ERRORLEVEL% NEQ 0 GOTO ERROR

echo.
@echo Reset the device. After reset it should enumerate as the bootloader.
sam-ba_3.3.1\sam-ba.exe -p serial:%1 -d samv71 -a reset
if %ERRORLEVEL% NEQ 0 GOTO ERROR

if [%2]==[] goto SUCCESS
echo.
echo Wait for the MBABootldr to enumerate.
ping localhost -n 10

echo.
@echo Flash the application.
Bootldr_flasher.exe MBA.bin
if %ERRORLEVEL% NEQ 0 GOTO ERROR

goto SUCCESS

:ERROR
echo.
echo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ERRORS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
goto END

:SUCCESS
echo.
echo Flash complete - success

:END

Endlocal
