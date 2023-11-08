pushd %~dp0

echo Generating protocol buffer C files...
protoc --plugin=protoc-gen-nanopb=..\nanopb\generator\protoc-gen-nanopb.bat --proto_path=..\nanopb\generator\proto --proto_path=. --nanopb_out=. .\aba.api.proto .\aba.can.proto .\aba.eeprom.proto
if NOT %ERRORLEVEL% == 0 goto FAILED

goto DONE
echo.
got DONE

:FAILED
echo Failed!

:DONE
popd
