How to Write a Tutorial
=======================

Tutorials are one of the most useful contributions you can make because they are the first thing many new users see.

Learning Objectives
-------------------
- Proper formatting for tutorials in the MoveIt documentation section.

Requirements
------------
- Ubuntu 20.04
- ROS 2 Galactic
- MoveIt2

Steps
-----

1 Copy the template
~~~~~~~~~~~~~~~~~~~

Create a new directory in the ``doc/tutorials`` directory, then create a file in that directory with the ``.rst`` extension with this template:

.. code-block::

  <Title>
  -------

  <Brief description of the tutorial with image showing what will be accomplished>

  Background
  ----------

  <Explination of what the user should have already done before this tutorial>

  Tasks
  -----

  1 <First Step>
  ~~~~~~~~~~~~~~

  <This should describe an action that the user should take such as creating a ROS project or typing up some code>

  1.1 <Explination First Step>
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  <Use substeps like this to walk the user through an explination of what they did>

  1.2 <Action First step>
  ^^^^^^^^^^^^^^^^^^^^^^^

  <Use a substep like this to describe running the new code and what the results should be>

  2 <Second Step>
  ~~~~~~~~~~~~~~~

  <...>

  Conclusion
  ----------

  <Here is where you explain what the user has read and provide additional references.>

  Next Step
  ---------

  <Link to the next tutorial here>

2 Write the tutorial
~~~~~~~~~~~~~~~~~~~~

Fill in the various sections.
It can be really helpful to do the tutorial yourself after you have written it to make sure everything in it works.
Make sure you add a link to your tutorial in the table of contents on the :doc:`/doc/tutorials/tutorials` page.
Lastly, submit your new tutorial as a PR and get feedback from others.

Further Reading
---------------

- Concepts page: :doc:`/doc/concepts/tutorials`

Next Step
---------

- Write a Tutorial
