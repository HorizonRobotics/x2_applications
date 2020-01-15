#!/usr/bin/sh

hbdk-cc --march x2 -m ./model-wo-bn-b8bit-pred-X2_1.4.json -p ./model-wo-bn-b8bit-pred-0033.params  -s 1x112x112x3 --pyramid-stride 112 -o faceID.hbm -i pyramid   --O2
