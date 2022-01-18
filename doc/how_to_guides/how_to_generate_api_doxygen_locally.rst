How To Generate API Doxygen Reference Locally
=============================================

Run doxygen in the root of the moveit2 repository.


This explains what how to generate doxygen reference locally:

.. code-block:: bash

  cd ~/ws_moveit/src/moveit2

.. code-block:: bash

  doxygen

It will generate a /doc/* directory containing the documentation.

The documentation entrypoint in a browser is index.html which you can access with:
.. code-block:: bash

  firefox ~/ws_moveit/src/moveit2/doc/index.html
