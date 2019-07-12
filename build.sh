#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
time make -j${CPUS}
