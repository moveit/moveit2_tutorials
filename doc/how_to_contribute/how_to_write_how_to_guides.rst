How to Write a MoveIt How-To Guide
==================================

This guide explains how to write How-To's for MoveIt documentation. This guide is primarily intended for use by employees at PickNik Robotics
to assist in standardizing how-to documents but can be used by any contributor who wants to submit a new guide. If you are looking for
how-to guides for using MoveIt, you can find them here: :doc:`/doc/how_to_guides/how_to_guides`.

Learning Objectives
-------------------
- The type of information that should be included in a how-to.
- Proper formatting for how-to guides in the MoveIt documentation section.

Requirements
------------
- Ubuntu 20.04
- ROS 2 Galactic
- MoveIt 2

Steps
-----

1. Fork the `MoveIt 2 Tutorials repository <https://github.com/moveit/moveit2_tutorials.git>`_ and start a new branch with an intuitive name (e.g., ``jack/how-to-write-how-tos``).

#. Create a new file in the ``doc/how_to_guides`` directory with a ``.rst`` extension. The title should start with "How to" followed by the specific question being answered (e.g., "How to Write a MoveIt How-To Guide").

#. Add a link to your guide on the appropriate how-to guide page:

   - :doc:`User Guides </doc/how_to_guides/how_to_guides>`

   - :doc:`Contributor Guides </doc/how_to_contribute/how_to_contribute>`

#. Write the introduction using reStructuredText (.rst) using the following guidelines:

   - The title should have the same name as the file.

   - The introduction should explain the purpose of this how-to and the intended audience.

   - If you think people may regularly find this particular guide by mistake, add links to the proper resource.

#. Write the specific learning objectives (i.e., what the reader will know when they finish reading this how-to).

#. Add any system or equipment requirements for this how-to so users know if this how-to is appropriate for them.

#. Lay out the individual action steps and do not leave out necessary intermediate steps.

#. Create a "Further Reading" section that links to amplifying information.

#. Submit the new page as a PR to the `MoveIt 2 Tutorials repository <https://github.com/moveit/moveit2_tutorials.git>`_.

Template
--------

.. code-block::

  <Title>
  -------

  <Brief description of the How-To Guide with image or GIF showing the outcome.>

  Learning Objectives
  -------------------

  <List of things the user will learn.>

  Requirements
  ----------

  <Explanation of what the user should understand. Unlike a tutorial, these guides stand alone and can assume the user has much more background.>

  Steps
  -----

  <A list of steps to take to solve the problem.>

  Further Reading
  ---------------

  <A list of links to related content on and off this website.>

Further Reading
---------------
- :doc:`/doc/how_to_contribute/how_to_contribute_to_site`
- :doc:`MoveIt Concepts: How-To Guide </doc/how_to_guides/how_to_guide>`
