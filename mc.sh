#!/usr/bin/env bash
rm -rf build
mkdir build && cd $_
cmake .. -DPICO_BOARD=pico_w
#cmake ..
make -j4
