#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /opt/workspace/
catkin build -DCMAKE_BUILD_TYPE="Release" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cat `find ./ -name compile_commands.json` > build/compile_commands.json && sed -i -e ':a;N;$!ba;s/\]\n*\[/,/g' build/compile_commands.json
compdb -p build list > compile_commands.json
echo "Done compile."
cd -
exit 0
