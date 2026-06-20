#!/bin/bash
# Note that a virtual environment is required when running this script in CI
#
# Usage: ./htmlproofer.sh [ros_distro]
#   ros_distro: ROS distro to build docs against (e.g. rolling, jazzy, humble).
#               Defaults to the ROS_DISTRO environment variable, or "rolling".
set -e

# Resolve the target ROS distro from the first argument, the ROS_DISTRO env var, or fall back to rolling
ROS_DISTRO_ARG=${1:-${ROS_DISTRO:-rolling}}

# Map the ROS distro to the corresponding moveit2 git branch.
# main serves both rolling and jazzy; humble has its own branch.
case "$ROS_DISTRO_ARG" in
  humble) MOVEIT_BRANCH=humble ;;
  *)      MOVEIT_BRANCH=main ;;
esac

# Define some config vars
export NOKOGIRI_USE_SYSTEM_LIBRARIES=true
export REPOSITORY_NAME=${PWD##*/}
export MOVEIT_BRANCH
echo "Testing branch $MOVEIT_BRANCH of $REPOSITORY_NAME (ROS distro: $ROS_DISTRO_ARG)"

# Install htmlpoofer
gem install --user-install html-proofer -v 3.19.4 # newer 4.x requires different cmdline options
PATH="$(ruby -r rubygems -e 'puts Gem.user_dir')/bin:$PATH"

# Install python dependencies
pip3 install --upgrade -r requirements.txt

# Clear out any previous builds
rm -rf build

# Build API docs
mkdir -p build/html
pushd build/html
git clone https://github.com/moveit/moveit2 -b $MOVEIT_BRANCH --depth 1
pushd moveit2
sed -i "s/HTML_EXTRA_STYLESHEET  =.*/HTML_EXTRA_STYLESHEET  = ..\/..\/..\/theme.css/g" Doxyfile
DOXYGEN_OUTPUT_DIRECTORY="../api" doxygen
popd
rm -rf moveit2
popd

# Test build with non-ROS wrapped Sphinx command to allow warnings and errors to be caught
# TODO: Re-add the -W flag so that all warnings are treated as errors.
sphinx-build -b html . build/html

# Replace Edit on Github links with local file paths
grep -rl 'https:\/\/github.com\/moveit\/moveit2_tutorials\/blob\/main\/' ./build/ | \
 xargs sed -i "s|https://github.com/moveit/moveit2_tutorials/blob/main/|file://$PWD|g"

# Replace internal links with local file paths
grep -rl "https:\/\/moveit.picknik.ai\/${ROS_DISTRO_ARG}\/" ./build/ | \
 xargs sed -i "s|https://moveit.picknik.ai/${ROS_DISTRO_ARG}/|file://$PWD|g"

# Run HTML tests on generated build output to check for 404 errors, etc
# 429 or 403 - happens when GitHub rate-limits requests
htmlproofer ./build \
  --only-4xx --check-html --http-status-ignore "429" --http-status-ignore "403" \
  --file-ignore ./build/html/genindex.html,./build/html/search.html,/html/api/ \
  --alt-ignore '/.*/' --url-ignore '#'

# Tell GitHub Pages (on deploy) to bypass Jekyll processing
touch build/.nojekyll
