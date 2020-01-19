#!/bin/bash

rm lib/libbbox_filter.so
cp lib/*.so ../smartplugin/deploy/xppcp_smart/lib/
cp bin/xppcp_smart ../smartplugin/deploy/xppcp_smart/
cd ../smartplugin/deploy
tar czf xppcp_smart.tgz xppcp_smart/ 
cd ../../build
mv ../smartplugin/deploy/xppcp_smart.tgz .