How-To Contribute to This Site
==============================

This explains how to make and submit changes to this site.
This how-to guide assumes you are on Ubuntu 20.04.

Setting up your development environment
---------------------------------------

Follow the tutorial :doc:`/doc/tutorials/getting_started/getting_started` to set up your workspace for testing local changes.

All further instructions assume you are the directory containing a local checkout of ``moveit2_tutorials``.

Build and view the website locally
----------------------------------

To build the website locally run this command:

.. code-block:: bash

  make html

Then you can use this to open the site in your webbrowser:

.. code-block:: bash

  open ./build/html/index.html

Test for broken links
---------------------

To test for broken links run this command:

.. code-block:: bash

  ./htmlprofer.sh

Run the formatters and spellchecker
-----------------------------------

We use `pre-commit <https://pre-commit.com/>`_ to run the formatting and spelling checkers.
You can install it with pip like this:

.. code-block:: bash

  pip3 install -U pre-commit

To run pre-commit locally to fix formatting and spelling:

.. code-block:: bash

  pre-commit run --all

Run industrial_ci locally to run CI
-----------------------------------

1. Clone a copy of `industrial_ci <https://github.com/ros-industrial/industrial_ci>`_ into your workspace.
2. Build and source your workspace.
3. Run this command from the workspace directory to test code changes just as it is done in CI:

.. code-block:: bash

  ros2 run industrial_ci rerun_ci src/moveit2_tutorials \
    DOCKER_IMAGE='moveit/moveit2:rolling-source' \
    UPSTREAM_WORKSPACE='moveit2_tutorials.repos' \
    TARGET_CMAKE_ARGS='-DCMAKE_BUILD_TYPE=Release' \
    CCACHE_DIR="$HOME/.ccache" \
    CLANG_TIDY='true'

Next Step
---------

:doc:`how_to_write_tutorials`
