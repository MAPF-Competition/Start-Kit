#!/bin/bash

mkdir -p build

cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j

# Runtime Python component selection via CLI flags:
#   ./build/lifelong -i <input.json>                           # all C++ default
#   ./build/lifelong -i <input.json> --plannerPython true      # Python planner only
#   ./build/lifelong -i <input.json> --schedulerPython true    # Python scheduler only
#   ./build/lifelong -i <input.json> --executorPython true     # Python executor only
#   ./build/lifelong -i <input.json> --plannerPython true --schedulerPython true --executorPython true  # all Python
