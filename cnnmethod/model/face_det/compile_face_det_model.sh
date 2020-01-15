#!/usr/bin/sh
hbdk-cc --march x2 -m ./multitask-1-absorb-bn-ft-1-x2-hobot-predict-symbol-x2-0.3.json -p ./multitask-1-absorb-bn-ft-1-x2-hobot-predict-0010-x2-0.3.params  -s 1x540x960x3 --pyramid-stride 960 -o faceDet.hbm -i pyramid   --O2
