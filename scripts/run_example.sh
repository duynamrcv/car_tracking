#!/bin/sh

./scripts/build.sh || { echo '*********************** Build failed! ***********************' ; exit 1; }

export LD_LIBRARY_PATH=$(pwd)/vendor/opencv/lib
./build/debug/bin/example 1 data/basement1.txt data/trajectory1.csv
