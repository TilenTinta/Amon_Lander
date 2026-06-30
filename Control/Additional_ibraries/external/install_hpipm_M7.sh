#!/bin/bash
set -e

BLASFEO_PATH=/home/tinta/Amon_lander/Additional_ibraries/external/blasfeo

cd /home/tinta/Amon_lander/Additional_ibraries/external

# Clean and reclone
rm -rf hpipm
git clone https://github.com/giaf/hpipm.git hpipm
cd hpipm

cat <<EOF > Makefile.local
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar
TARGET=GENERIC
BLASFEO_PATH=$BLASFEO_PATH
CFLAGS=-O2 \
       -mcpu=cortex-m7 \
       -mthumb \
       -mfloat-abi=hard \
       -mfpu=fpv5-d16 \
       -ffast-math \
       -ffunction-sections \
       -fdata-sections \
       -DBLASFEO_USE_SINGLE_PRECISION
EOF

make static_library TARGET=GENERIC BLASFEO_PATH=$BLASFEO_PATH -j$(nproc)

# Verify immediately
echo "=== Verifying HPIPM ==="
file lib/libhpipm.a
objdump -f lib/libhpipm.a | grep -m1 "architecture"
