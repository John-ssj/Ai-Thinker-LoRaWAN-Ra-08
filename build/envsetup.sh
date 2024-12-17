#/bin/sh

export TREMO_SDK_PATH=$(pwd)

GNU_ARM_EMBEDDED_TOOLCHAIN_URL='https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2?rev=78196d3461ba4c9089a67b5f33edf82a&hash=D484B37FF37D6FC3597EBE2877FB666A41D5253B'

which arm-none-eabi-gcc >/dev/null 2>&1
if [ $? -ne 0 ]; then
    if [ ! -e $TREMO_SDK_PATH/tools/toolchain/bin/arm-none-eabi-gcc ]; then
        if [ ! -e $TREMO_SDK_PATH/tools/toolchain/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 ]; then
            wget $GNU_ARM_EMBEDDED_TOOLCHAIN_URL -O $TREMO_SDK_PATH/tools/toolchain/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
        fi
        cd ./tools/toolchain/
        tar -jxvf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 --strip-components=1
        cd -
    fi

    which arm-none-eabi-gcc >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        export PATH="$PATH:$TREMO_SDK_PATH/tools/toolchain/bin/"
    fi
fi
