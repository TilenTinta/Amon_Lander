#!/bin/bash
set -e
echo "=== Installing BLASFEO for Cortex-M7 ==="

git clone https://github.com/giaf/blasfeo.git
cd blasfeo
make clean || true

cat <<EOF > Makefile.local
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

# GENERIC is correct — no M-profile asm kernels exist in blasfeo
TARGET=GENERIC
LA=REFERENCE

CFLAGS=-O2 \
       -mcpu=cortex-m7 \
       -mthumb \
       -mfloat-abi=hard \
       -mfpu=fpv5-d16 \
       -ffast-math \
       -ffunction-sections \
       -fdata-sections

EOF

make static_library -j$(nproc)
echo "=== BLASFEO built for Cortex-M7 ==="
