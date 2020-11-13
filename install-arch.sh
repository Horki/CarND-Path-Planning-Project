#! /bin/bash

set -ex

sudo pacman -S openssl zlib
git submodule update --init
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
