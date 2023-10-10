TRAC-IK Kinematics Solver
=========================

`TRAC-IK <https://bitbucket.org/traclabs/trac_ik>`_ is an inverse kinematics solver developed by TRACLabs that combines two IK implementations via threading to achieve more reliable solutions than common available open source IK solvers.
From their documentation:

  (TRAC-IK) provides an alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL.
  Specifically, KDL's convergence algorithms are based on Newton's method, which does not work well in the presence of joint limits --- common for many robotic platforms.
  TRAC-IK concurrently runs two IK implementations.
  One is a simple extension to KDL's Newton-based convergence algorithm that detects and mitigates local minima due to joint limits by random jumps.
  The second is an SQP (Sequential Quadratic Programming) nonlinear optimization approach which uses quasi-Newton methods that better handle joint limits.
  By default, the IK search returns immediately when either of these algorithms converges to an answer.
  Secondary constraints of distance and manipulability are also provided in order to receive back the "best" IK solution.

The package `trac_ik_kinematics_plugin <https://bitbucket.org/traclabs/trac_ik/src/rolling-devel/trac_ik_kinematics_plugin/>`_ provides a ``KinematicsBase`` MoveIt interface that can replace the default KDL solver.
Currently, mimic joints are *not* supported.

Install
-------

The ``rolling-devel`` branch of the TRAC-IK repository has the latest ROS 2 implementation.
For now, the repository must be built from source in your ROS 2 workspace, for example ``~/moveit2_ws``. ::

  cd ~/moveit2_ws/src
  git clone -b rolling-devel https://bitbucket.org/traclabs/trac_ik.git

Usage
-----

- Find the MoveIt :doc:`kinematics.yaml </doc/examples/kinematics_configuration/kinematics_configuration_tutorial>` file created for your robot.
- Replace ``kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin`` (or similar) with ``kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin``
- Make sure you add a ``<depend>trac_ik_kinematics_plugin</depend>`` tag to your package's corresponding ``package.xml`` file.
- Set parameters as desired:

  - **kinematics\_solver\_timeout** (timeout in seconds, e.g., ``0.005``) and **position\_only\_ik** **ARE** supported.
  - **solve\_type** can be ``Speed``, ``Distance``, ``Manip1``, ``Manip2`` (see below for details). Defaults to ``Speed``.
  - **epsilon** is the Cartesian error distance used to determine a valid solution. Defaults to ``1e-5``.
  - **kinematics\_solver\_attempts** parameter is unneeded: unlike KDL, TRAC-IK solver already restarts when it gets stuck.
  - **kinematics\_solver\_search\_resolution** is not applicable here.

From the `trac_ik_lib <https://bitbucket.org/traclabs/trac_ik/src/rolling-devel/trac_ik_lib/>`_ package documentation (slightly modified), here is some information about the solve type parameter:

  - ``Speed``: returns very quickly the first solution found.
  - ``Distance``: runs for the full timeout, then returns the solution that minimizes sum of squares error (SSE) from the seed.
  - ``Manip1``: runs for full timeout, returns solution that maximizes ``sqrt(det(J*J^T))`` (the product of the singular values of the Jacobian).
  - ``Manip2``: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
