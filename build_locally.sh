#!/bin/sh

# Setup Environment
rm -rf build

# Build
sphinx-build -W -b html . build

# Run
xdg-open ./build/index.html