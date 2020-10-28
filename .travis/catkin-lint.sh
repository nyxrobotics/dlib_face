#!/bin/bash

if [ `catkin_lint 2>/dev/null | tee /dev/stderr | wc -l` -gt 0 ]; then
  echo "Error: Invalid code in CMakeLists.txt or package.xml" >&2
  exit 1
fi

echo "Done catkin-lint validation."
exit 0
