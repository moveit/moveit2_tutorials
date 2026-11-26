How To Generate API Doxygen Reference Locally
=============================================


Run doxygen in the root of the moveit2 repository.

Requirements
------------

- Install ``doxygen`` and ``graphviz``:

.. code-block:: bash

    sudo apt-get install doxygen graphviz

Steps
-----

- Navigate to the moveit2 repository:

.. code-block:: bash

  cd ~/ws_moveit/src/moveit2

- Run reference generation command with desired output directory path:

.. code-block:: bash

  DOXYGEN_OUTPUT_DIRECTORY=~/docs doxygen

- The documentation entrypoint in a browser is index.html which you can access with:

.. code-block:: bash

  firefox ~/docs/index.html
