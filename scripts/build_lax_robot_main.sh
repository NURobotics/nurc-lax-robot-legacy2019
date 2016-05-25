#!/usr/bin/env bash

TARGET_BUILD_FOLDER=../build
TARGET_TOOLCHAIN_FILE=../tools/toolchain-arm-linux-gnueabi.cmake

mkdir $TARGET_BUILD_FOLDER

cd $TARGET_BUILD_FOLDER
cmake -DCMAKE_TOOLCHAIN_FILE=../$TARGET_TOOLCHAIN_FILE ../
