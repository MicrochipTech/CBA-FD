@echo off

IF NOT DEFINED ABA_MSVC (
  echo [31mABA Environment for Microsoft Visual Studio 2019 not set! Please provide a proper ABA_SETUP.CMD[0m
  exit /B
)

set INCLUDE=%ABA_MSVC%\Windows Kits\10\include\10.0.18362.0\ucrt;%ABA_MSVC%\Windows Kits\10\include\10.0.18362.0\shared;%ABA_MSVC%\Windows Kits\10\include\10.0.18362.0\um;%ABA_MSVC%\2019\BuildTools\VC\Tools\MSVC\14.27.29110\include;
set LIB=%ABA_MSVC%\2019\BuildTools\VC\Tools\MSVC\14.27.29110\lib\x64;%ABA_MSVC%\Windows Kits\10\lib\10.0.18362.0\ucrt\x64;%ABA_MSVC%\Windows Kits\10\lib\10.0.18362.0\um\x64;
set path=%path%;%ABA_MSVC%\2019\BuildTools\VC\Tools\MSVC\14.27.29110\bin\Hostx64\x64