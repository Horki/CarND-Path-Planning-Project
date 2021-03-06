#! /bin/bash

set -ex

brew install openssl libuv cmake zlib
git submodule update --init
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
