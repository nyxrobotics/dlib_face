#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
for f in $(git ls-files | grep -E '*\.(cpp|h)$'); do
  if [ `rosrun roslint cpplint "${f}" | tee /dev/stderr | wc -l` -gt 2 ]; then
    echo "Error: Invalid code in ${f}" >&2
    exit 1
  fi
done
for f in $(git ls-files | grep -E '*\.(py)$'); do
  if [ `rosrun roslint pep8 "${f}" | tee /dev/stderr | wc -l` -gt 0 ]; then
    echo "Error: Invalid code in ${f}" >&2
    exit 1
  fi
done
echo "Done roslint validation."
exit 0
