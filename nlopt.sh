#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"/nlopt
cd  $SCRIPT_DIR
mkdir -p build
cd build
cmake ..
make
sudo make install