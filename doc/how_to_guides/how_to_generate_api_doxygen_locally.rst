How To Generate API Doxygen Reference Locally
=============================================

Run doxygen in the root of the moveit2 repository.


This explains what how to generate doxygen reference locally:

1. Install doxygen:

.. code-block:: bash

    sudo apt install doxygen
    
2. Navigate to the moveit2 repository:

.. code-block:: bash

  cd ~/ws_moveit/src/moveit2

3. Run reference generation command with desired output directory path:

.. code-block:: bash

  DOXYGEN_OUTPUT_DIRECTORY=~/docs doxygen

4. The documentation entrypoint in a browser is index.html which you can access with:

.. code-block:: bash

  firefox ~/docs/index.html
