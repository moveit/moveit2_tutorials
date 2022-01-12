How to Write a Tutorial
=======================

Tutorials are one of the most useful contributions you can make because they are the first thing many new users see.

Background
----------

We have separated the sections of this website based on `The Documentation System <https://documentation.divio.com/>`_.
Tutorials are **Learning-Oriented** and have these qualities:

* learning by doing
* inspiring confidence
* repeatability
* immediate sense of achievement
* concretness, not abstraction
* minimum necessary explanation
* no distractions
* no options or decisions

In the end, the reader should feel a sense of accomplishment.
For that, they have to take some actions that they were instructed to.
We should avoid the pattern of having a pre-made program that the user just executes followed by an explanation of the code.
Ideally each tutorial should extend the previous one to make them feel connected.

Tutorials are not the place for explaining nuanced abstractions or discussing decisions.
In order to foster learning by the newest users they should build muscle memory around using the varous components in MoveIt to do specific tasks.

When a user is going through the tutorials, we should avoid distracting them with potential rabbit holes as that'll detract from the experience of the tutorials.
Save detailed explinations for the :doc:`/doc/concepts/concepts` pages on this site.
Place links to external sties and other resources at the end of the tutorial.

It is important that the tutorials are reviewed by those that are as close as possible to the intended audience in their amount of experience with MoveIt.

Awesome Examples
~~~~~~~~~~~~~~~~

Here are some awesome examples to draw inspiration from for creating tutorials:

* `ROS 1 Tutorials <http://wiki.ros.org/ROS/Tutorials>`_
* `The Rust Book <https://doc.rust-lang.org/book/>`_
* `CUDA Toolkit <https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#introduction>`_
* `libuv book <http://docs.libuv.org/en/v1.x/guide/introduction.html>`_
* `React Native Docs <https://reactnative.dev/docs/getting-started>`_

Tasks
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

Next Step
---------

:doc:`how_to_write_how_to_guides`
