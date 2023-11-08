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

@REM Import paths from the toolchain configuration
set path=%ABA_PYTHON%;%ABA_PROTOC%;%WINDIR%;%WINDIR%\system32

@REM Print paths as debug for the CI job results
echo.
echo Listing path configuration
echo %path%
echo.
echo Python Path:
@where python.exe

call Common\Generated\generate.cmd

rmdir /s /q Debug\Protobuf
xcopy Common\Generated\*.pb.* Debug\Protobuf\


EndLocal
