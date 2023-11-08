@echo off

SetLocal

@REM CI job runs the script from the repository root directory, so enter SW folder
cd SW 2>nul

@REM Check if the toolchain configuration have been setup properly
IF NOT DEFINED ABA_SETUP (
  call ABA_SETUP.cmd
  IF NOT DEFINED ABA_SETUP (
    echo [31mABA Environment variables not set! Please provide a falid ABA_SETUP.CMD[0m
	exit /B
  )
)

IF /I "%1" == "Release" (
  echo [33mRelease build selected[0m
  set ABA_CFG=Release
	
) ELSE (
  echo [33mDebug build selected[0m
  set ABA_CFG=Debug
)

@REM Get commit hash from code and store it in environment variable
for /f "tokens=*" %%a in ('git log --pretty^=format:%%h -n 1') do set MBA_COMMIT_ID^=%%a

@REM Import paths from the toolchain configuration
set path=%ABA_RUBY%;%ABA_MINGW%;%ABA_ATARM%;%ABA_XC16%;%ABA_PYTHON%;%ABA_PROTOC%;%WINDIR%;%WINDIR%\system32
call %ABA_MSVC%\msvc_setup.cmd

@REM Print paths as debug for the CI job results
echo.
echo [92mListing path configuration[0m
echo %path%
echo.
echo MinGW GCC Path:
@where gcc.exe
echo MSVC Path:
@where cl.exe
echo ARM GCC Path:
@where arm-none-eabi-gcc.exe
echo Protobuf Path:
@where protoc.exe
echo Python Path:
@where python.exe
echo Ruby/Bakery Path:
@where bakery.bat
echo.

echo.
echo [92mCall bakery to build firmware and drivers[0m
call bakery -b %ABA_CFG% --rebuild -a black
if %ERRORLEVEL% NEQ 0 GOTO ERROR

goto SUCCESS

:ERROR
echo.
echo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ERRORS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
goto END

:SUCCESS
echo.
echo Build complete - success

:END

EndLocal
