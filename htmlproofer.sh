#!/bin/bash
set -e

# Define some config vars
export NOKOGIRI_USE_SYSTEM_LIBRARIES=true
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"

# Install htmlpoofer
gem install --user-install html-proofer
PATH="$(ruby -r rubygems -e 'puts Gem.user_dir')/bin:$PATH"

# Install python dependencies
pip3 install --user --upgrade -r requirements.txt

# Clear out any previous builds
rm -rf build

# Test build with non-ROS wrapped Sphinx command to allow warnings and errors to be caught
sphinx-build -W -b html . build

# Run HTML tests on generated build output to check for 404 errors, etc
[[ -v GITHUB_HEAD_REF ]] || URL_SWAP="--url-swap https\://github.com/ros-planning/moveit2_tutorials/blob/main/:file\://$PWD/build/"
htmlproofer ./build --only-4xx --check-html --file-ignore ./build/genindex.html,./build/search.html --alt-ignore '/.*/' --url-ignore '#' $URL_SWAP

# Tell GitHub Pages (on deploy) to bypass Jekyll processing
touch build/.nojekyll
