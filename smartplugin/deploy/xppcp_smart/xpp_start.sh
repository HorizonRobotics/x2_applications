#!/bin/sh
project_path=$(cd `dirname $0`; pwd)
echo $project_path
old_dir=`pwd`
echo $old_dir
cd $project_path
export LD_LIBRARY_PATH=$project_path/lib/:$LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH
#nohup ./xppcp_smart configs/smart_config.json >/dev/null 2>&1 &
./xppcp_smart  configs/smart_config.json
cd $old_dir
