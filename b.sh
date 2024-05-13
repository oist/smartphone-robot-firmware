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
cd ..
