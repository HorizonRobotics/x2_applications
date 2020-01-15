#! /usr/bin/sh

hbdk-cc --march x2 -m /local/model_compile/models/lmk_pose/hobot-predict-20190403-symbol.json -p /local/model_compile/models/lmk_pose/hobot-predict-20190403-0210.params  -s 1x64x64x3  -o facePoseLMKs.hbm -i resizer --O2
