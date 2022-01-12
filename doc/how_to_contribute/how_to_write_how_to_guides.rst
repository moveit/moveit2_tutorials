How to Write a How-To Guide
===========================

The How-To Guides are distinctly different from tutorials in that they are directed towards a different audience.

Background
----------

You should assume much more knoledge of MoveIt and simply offer a quick guide for a specific problem.
These pages posses these qualities:

* problem oriented
* a series of steps
* a focus on the goal
* addressing a specific question
* no unnecessary explanation
* a little flexibility
* practical usability
* good naming

In a tutorial you carefully guide the reader through doing a set steps with the goal of learning.
Readers who are following tutorials don't know the terms or concepts yet to ask specific questions.
Readers of How-To guides are already using MoveIt but are looking for isntructions on doing something specific.

Names Answer a User's Question
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How-To guides answer very specific questions and should capture Google searches.
For that reason the naming of them is really important.
An example of some potentially good names are:

* How to Visualize Collisions in MoveIt
* How to Grasp Objects with Moveit
* How to run MoveIt with UR5
* How to Fix a Segfault
* How to Migrate from Foxy to Galactic
* How to Run in Gazebo
* How to Setup a New Robot for MoveIt
* How to use the MoveIt RViz plugin
* How to Teleop a Robot Arm with a Controller

Goal Focused and Practical
~~~~~~~~~~~~~~~~~~~~~~~~~~

For these guides it is really important that you focus specifically on solving the problem the user came to the site for.
Because these users have a deeper understanding though you should present the user with some options when they will need to make a decision with tradeoffs.
You should state what the assumed pre-requisites are in the introduction to the How-To Guide.
That way you can avoid having multiple copies of isntructions on this site and allow users who already know the prerequisites to jump right into solving their problem.

Steps
-----

1 Copy the template
~~~~~~~~~~~~~~~~~~~

Create a new file in the ``doc/how_to_guides`` directory with the ``.rst`` extension with this template:

.. code-block::

  <Title>
  -------

  <Brief description of the How-To Guide with image or GIF showing the outcome>

  Background
  ----------

  <Explination of what the user should understand.  Unlike a tutorial these are stand-alone and can assume the user has much more background.>

  Steps
  -----

  1 <First Step>
  ~~~~~~~~~~~~~~

  <The first thing you need to do.>

  2 <Second Step>
  ~~~~~~~~~~~~~~~

  <...>

  Conclusion
  ----------

  <Here you should show the result of the How-To guide and provide references for further reading.>

2 Write the How-To Guide
~~~~~~~~~~~~~~~~~~~~~~~~

Fill in the various sections.
Remember your goal here is to help someone solve a specific problem.
Excellent How-To Guides should be short and to the point.
Make sure you add a link to your How-To Guide in the table of contents on the :doc:`/doc/how_to_guides/how_to_guides` page.
Lastly, submit your new page as a PR and get feedback from others.

Next Step
---------

Write a How-To Guide
