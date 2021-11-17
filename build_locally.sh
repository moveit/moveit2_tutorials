#!/bin/sh

# Install dependencies
pip3 install --user --upgrade -r requirements.txt

# Setup Environment
rm -rf build

# Build
make html

# Run
xdg-open ./build/html/index.html
