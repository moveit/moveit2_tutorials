#!/bin/bash
set -e

# Define some config vars
export NOKOGIRI_USE_SYSTEM_LIBRARIES=true
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"

# Install htmlpoofer
gem install --user-install html-proofer -v 3.19.4 # newer 4.x requires different cmdline options
PATH="$(ruby -r rubygems -e 'puts Gem.user_dir')/bin:$PATH"

# Install python dependencies
pip3 install --user --upgrade -r requirements.txt

# Clear out any previous builds
rm -rf build

# Test build with non-ROS wrapped Sphinx command to allow warnings and errors to be caught
sphinx-build -W -b html . build

# Replace Edit on Github links with local file paths
grep -rl 'https:\/\/github.com\/ros-planning\/moveit2_tutorials\/blob\/main\/' ./build/ | \
 xargs sed -i "s|https://github.com/ros-planning/moveit2_tutorials/blob/main/|file://$PWD|g"

# Replace internal links with local file paths
grep -rl 'https:\/\/moveit.picknik.ai\/humble\/' ./build/ | \
 xargs sed -i "s|https://moveit.picknik.ai/humble/|file://$PWD|g"

# Run HTML tests on generated build output to check for 404 errors, etc
htmlproofer ./build \
  --only-4xx --check-html --http-status-ignore "429" \
  --file-ignore ./build/genindex.html,./build/search.html \
  --alt-ignore '/.*/' --url-ignore '#'

# Tell GitHub Pages (on deploy) to bypass Jekyll processing
touch build/.nojekyll
