#!/bin/bash

mkdir build

# build exec

cd build
cmake ./
make -j
