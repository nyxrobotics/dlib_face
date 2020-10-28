#!/bin/bash

for f in $(git ls-files | grep -E '*\.(cpp|h)$'); do
  if [ `clang-tidy-10 "${f}" 2>/dev/null | tee /dev/stderr | wc -l` -gt 0 ]; then
    echo "Error: Invalid code in ${f}" >&2
    exit 1
  fi
done
echo "Done lint validation."
exit 0
