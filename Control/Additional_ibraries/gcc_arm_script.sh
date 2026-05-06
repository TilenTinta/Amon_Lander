#!/bin/bash

set -e

sudo apt update
sudo apt install -y \
    gcc-arm-none-eabi \
    gdb-multiarch \
    cmake \
    make \
    git \
    build-essential

echo "Toolchain installed"
