#!/bin/sh

# Install sphinx-build if it isn't there yet
if ! type "sphinx-build" > /dev/null
then
  echo "Installing sphinx"
  pip install sphinx==1.8.5
fi

# Setup Environment
rm -rf build

# Build
sphinx-build -W -b html . build

# Run
xdg-open ./build/index.html
