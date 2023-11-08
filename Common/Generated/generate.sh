#!/bin/bash

pushd `pwd` > /dev/null

echo Generating versions files...
python generate_versions.py -l DEBUG

echo Generating protocol buffer files...
protoc --plugin=protoc-gen-nanopb=../nanopb/generator/protoc-gen-nanopb --proto_path=../nanopb/generator/proto --proto_path=.     --nanopb_out=. ./aba.api.proto ./aba.can.proto ./aba.sent.proto ./aba.cxpi.proto ./aba.lin.proto ./aba.eeprom.proto

if [ $? -eq 0 ]
then
	echo Done!
else
	echo Failed!
fi
printf "\n"
popd > /dev/null
