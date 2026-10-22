#!/bin/bash

set -e

mkdir -p buildall
rm -rf buildall/*

# Create build directory if it doesn't exist
mkdir -p build
rm -rf build/*
cd build

export PICO_SDK_PATH=$HOME/downloads/pico-sdk
export PICOTOOL_FETCH_FROM_GIT_PATH=$HOME/downloads/pico-sdk

cmake -DBOARD=MSP20K ..
make
mv fpga_companion.uf2 ../buildall/fpga_companion_MSP20K.uf2
make clean

cmake -DBOARD=PICO ..
make
mv fpga_companion.uf2 ../buildall/fpga_companion_PICO.uf2
make clean

cmake -DBOARD=WS2040ZERO ..
make
mv fpga_companion.uf2 ../buildall/fpga_companion_WS2040ZERO.uf2
make clean

rm -r *

export PICO_SDK_PATH=$HOME/downloads/pico-sdk2
export PICOTOOL_FETCH_FROM_GIT_PATH=$HOME/downloads/pico-sdk2

cmake -DBOARD=MSP2_20K ..
make
mv fpga_companion.uf2 ../buildall/fpga_companion_MSP2_20.uf2
make clean

cmake -DBOARD=PICO2 ..
make
mv fpga_companion.uf2 ../buildall/fpga_companion_PICO2.uf2
make clean

echo "Contents of buildall directory:"
ls -1 ../buildall/