#!/bin/bash
set -e

echo "=== Installing BLASFEO ==="

git clone https://github.com/giaf/blasfeo.git
cd blasfeo

# čist build
make clean || true

# ključne nastavitve za STM32
cat <<EOF > Makefile.local
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar
TARGET=GENERIC
LA=REFERENCE
CFLAGS=-O2 -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -ffast-math
EOF

make static_library

echo "BLASFEO built!"
cd ..
