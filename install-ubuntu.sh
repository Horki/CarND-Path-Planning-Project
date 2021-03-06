#! /bin/bash

set -ex

sudo apt-get install libuv1-dev libssl-dev libz-dev
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
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
