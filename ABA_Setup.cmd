@echo off

echo [33mUsing default ABA_SETUP file![0m

set ABA_TOOLCHAIN=YOUR_PATH_HERE


set ABA_ATARM=%ABA_TOOLCHAIN%\arm\arm-gnu-toolchain-12.2\bin
set ABA_ATBACKEND=%ABA_TOOLCHAIN%\Atmel\Studio\7.0.2397\atbackend
set ABA_ATMELSTUDIO=%ABA_TOOLCHAIN%\Atmel\Studio\7.0.2397
set ABA_MSVC=%ABA_TOOLCHAIN%\msvc
set ABA_PYTHON=%ABA_TOOLCHAIN%\Python\Python-3.11.3
set ABA_PROTOC=%ABA_TOOLCHAIN%\Google\protoc-23.2-win64\bin
set ABA_RUBY=%ABA_TOOLCHAIN%\Ruby\Ruby27-x64\bin

echo Following tools are marked as deprecated:
set ABA_MINGW=%ABA_TOOLCHAIN%\mingw-w64\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin
echo     [33m%ABA_MINGW%[0m

set ABA_SETUP=1

echo Toolchain directory: [33m%ABA_TOOLCHAIN%[0m
echo ABA project setup done!
