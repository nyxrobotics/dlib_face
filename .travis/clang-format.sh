#!/bin/bash

for f in $(git ls-files | grep -E '*\.(cpp|h)$'); do
  if ! clang-format-10 "${f}" | diff "${f}" - > /dev/null 2>&1; then
    echo "Error: Invalid code formatting in ${f}" >&2
    exit 1
  fi
done
echo "Done format validation."
exit 0
