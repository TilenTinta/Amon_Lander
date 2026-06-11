#!/bin/bash
set -e

echo "=== Installing HPIPM (FIXED) ==="

git clone https://github.com/giaf/hpipm.git hpipm || true
cd hpipm

make clean || true

# ABSOLUTNA pot do BLASFEO
BLASFEO_PATH=$(realpath ../blasfeo)

cat <<EOF > Makefile.local
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

TARGET=GENERIC
BLASFEO_PATH=$BLASFEO_PATH

CFLAGS=-O2 -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -ffast-math
EOF

# ključ: override TARGET pri make callu
make static_library TARGET=GENERIC BLASFEO_PATH=$BLASFEO_PATH

echo "HPIPM built successfully!"


#cd hpipm

#BLASFEO_PATH=$(realpath ../blasfeo)

#make -B static_library \
#  CC=arm-none-eabi-gcc \
#  AR=arm-none-eabi-ar \
#  TARGET=GENERIC \
#  BLASFEO_PATH="$BLASFEO_PATH" \
#  CFLAGS="-O2 -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 -ffast-math -I$BLASFEO_PATH/include -I$(pwd)/include -DTARGET_GENERIC -DUSE_C99_MATH -DEXT_DEP"
