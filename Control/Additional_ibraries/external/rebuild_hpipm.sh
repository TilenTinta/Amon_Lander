cd /home/tinta/Amon_lander/Additional_ibraries/external/hpipm

make clean

BLASFEO_PATH=/home/tinta/Amon_lander/Additional_ibraries/external/blasfeo
HPIPM_PATH=/home/tinta/Amon_lander/Additional_ibraries/external/hpipm

make static_library \
    CC=arm-none-eabi-gcc \
    AR=arm-none-eabi-ar \
    TARGET=GENERIC \
    BLASFEO_PATH=$BLASFEO_PATH \
    CFLAGS="-O2 -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 -ffast-math -ffunction-sections -fdata-sections -I$BLASFEO_PATH/include -I$HPIPM_PATH/include" \
    -j$(nproc)

# Verify
file lib/libhpipm.a
objdump -f lib/libhpipm.a | grep -m1 "architecture"
