#!/usr/bin/env bash

rm -rf ../cpp/build
rm -rf ../cpp/bin
mkdir ../cpp/build

cd ../cpp/build/ &&
cmake .. &&
make &&
cd -