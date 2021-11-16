#!/bin/sh

# Install sphinx if it isn't there yet
if ! type "sphinx-multiversion" > /dev/null
then
  echo "Installing sphinx"
  pip install -r requirements.txt
fi

# Setup Environment
rm -rf build

# Build
make html

# Run
xdg-open ./build/html/index.html
