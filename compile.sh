#!/bin/bash

mkdir build

# build exec for cpp

cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j
