#!/bin/sh

cd external
rm -rf build
mkdir build
cd build
cmake ..
make -j8
cd ../..
rm -rf build
mkdir build
cd build
cmake ..
make -j8
USER_GROUP=$(stat -c "%u:%g" ..)
chown -R $USER_GROUP .
cd ..
