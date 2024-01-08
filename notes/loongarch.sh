#!/bin/csh
setenv TARGET loongarch
setenv TARGET_ARCH loongarch64
setenv SRCCONF /etc/src.conf
#setenv CROSS_TOOLCHAIN llvm17

cd /usr/src
make -j4 kernel-toolchain

