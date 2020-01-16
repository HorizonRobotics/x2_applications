#!/bin/bash
cmake -DCMAKE_C_COMPILTER=aarch64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ ..
make VERBOSE=1