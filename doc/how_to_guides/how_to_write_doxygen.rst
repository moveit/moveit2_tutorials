How to Contribute Doxygen Comments
----------------------------------


This guide will provide an introduction and overview to good practices for contributing Doxygen comments.

Learning Objectives
-------------------

- How to write helpful Doxygen Comments
- Some useful Doxygen Plugins

Instructions
------------
When making a contribution to MoveIt (or any code, really), make sure your code is readable and well-commented throughout.
Using Doxygen comments allows for standardization of documentation and ensures that all contributions have certain information included with them.
One of the primary benefits of Doxygen is that it allows for automatic generation of API documentation in a consistent, readable format.


Plugins exist to automate the creation of Doxygen documentation for
- `SublimeText <https://packagecontrol.io/packages/DoxyDoxygen>`_
- `VIM <https://www.vim.org/scripts/script.php?script_id=987>`_
- `VSCode <https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen>`_

As well as for `many other IDEs <https://www.doxygen.nl/helpers.html>`_.

In general, a Doxygen comment should include a brief description of the thing it is commenting on at the very least.
Descriptions of the input parameters (if any) as well as the output parameters (if any) is helpful as well.

Several examples are provided below:


    .. code-block:: c++

        /** @brief Check for robot self collision. Any collision between any pair of links is checked for, NO collisions are
        *   ignored.
        *
        *  @param req A CollisionRequest object that encapsulates the collision request
        *  @param res A CollisionResult object that encapsulates the collision result
        *  @param state The kinematic state for which checks are being made */
        virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                        const moveit::core::RobotState& state) const = 0;



    .. code-block:: c++

        /** @brief A bounding volume hierarchy (BVH) implementation of a tesseract contact manager */
        class BulletBVHManager
        {
        ...

These examples both serve to provide the types and descriptions of inputs and outputs and capture what the function or class is doing, in brief.


Further Reading
---------------

Feel free to look around in the repositories to see additional examples of Doxygen comments.
Looking at a similar piece of code to what you will be contributing and seeing its comments is the easiest way to learn.


See the how-to guide on how to generate Doxygen API locally :doc:`here <./how_to_generate_api_doxygen_locally>`.

See the Doxygen documentation guide `here <https://www.doxygen.nl/manual/docblocks.html>`_.
