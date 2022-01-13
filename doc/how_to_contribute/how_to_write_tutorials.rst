How to Write a MoveIt Tutorial
==============================

This guide explains how to write Tutorials for MoveIt documentation which are one of the most useful contributions you can make because they are the first thing many new users see.
This guide is primarily intended for use by employees at PickNik Robotics to assist in standardizing tutorial documents but can be used by any contributor who wants to submit a new tutorial.
If you are looking for how-to guides for using MoveIt, you can find them here: How-To Guides.

Learning Objectives
-------------------
- The type of information that should be included in a tutorial.
- Proper formatting for tutorials in the MoveIt documentation section.

Requirements
------------
- Ubuntu 20.04
- ROS 2 Galactic
- MoveIt2

Steps
-----

1. Fork the MoveIt2 Tutorials repository (https://github.com/ros-planning/moveit2_tutorials.git) and start a new branch with an intuitive name (e.g. jack/how-to-tutorials)

#. Create a new file in the ``doc/tutorials`` directory with a ``.rst`` extension. The title should start with a concise description and end with "Tutorial" (e.g. "Writing MoveIt Tutorials Tutorial").

#. Add a link to your tutorial on the tutorials page: :doc:`/doc/tutorials/tutorials`

#. Write the introduction using reStructuredText (.rst) using the following guidelines:

   - The title should have the same name as the file.

   - The introduction should explain the purpose of this tutorial and the intended audience.

   - If you think people may regularly find this particular guide by mistake, add links to the proper resource.

#. Write the specific learning objectives (i.e. what will the reader know when they finish reading this tutorial).

#. Add any system or equipment requirements for this tutorial so users know if this tutorial is appropriate for them.

#. Layout the interactive steps to be followed with enough details that the reader can easily follow along.

#. Write a conclusion to summarize the tutorial and provide additional resources.

#. Add a link to the next tutorial the reader should follow.

#. Submit the new page as a PR to the MoveIt2 Tutorials repository (https://github.com/ros-planning/moveit2_tutorials.git).

Template
--------

.. code-block::

  <Title>
  -------

  <Brief description of the tutorial with image showing what will be accomplished>

  Background
  ----------

  <Explination of what the user should have already done before this tutorial>

  Steps
  -----

  1. <First Step>
  ---------------

  <This should describe an action that the user should take such as creating a ROS project or typing up some code>

  1.1) <Explination First Step>
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  <Use substeps like this to walk the user through an explination of what they did>

  1.2) <Action First step>
  ^^^^^^^^^^^^^^^^^^^^^^^

  <Use a substep like this to describe running the new code and what the results should be>

  2. <Second Step>
  ----------------

  <...>

  Conclusion
  ----------

  <Here is where you explain what the user has read and provide additional references.>

  Next Step
  ---------

  <Link to the next tutorial here>

Further Reading
---------------
- :doc:`/doc/how_to_contribute/how_to_contribute_to_site`
- Concepts page: :doc:`/doc/concepts/tutorials`
