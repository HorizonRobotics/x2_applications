#!/usr/bin/sh

hbdk-cc --march x2 -m ./model_x2_infer-symbol.json -p ./model_x2_infer-0041.params  -s 1x128x128x1 --pyramid-stride 128 -o faceAntiSpfNIR.hbm -i pyramid   --O2
