#!/bin/bash

cd "$(dirname "$0")"

cd Common/Generated
./generate.sh
cd ../..

export PATH=/opt/arm-gnu-toolchain/bin:$PATH

bakery -b Linux_All --rebuild -a black
