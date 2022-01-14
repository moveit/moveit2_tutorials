Pick and Place with MoveIt Task Constructor
===========================================

(TBD - IMAGE HERE)

This tutorial will walk you through creating a package that plans a pick and place operation usin MoveIt Task Constructor. MoveIt Task Constructor provides a way to plan for tasks that consist of multiple different subtasks (known as stages).

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in :doc:`Getting Started </doc/tutorials/getting_started/getting_started>`.

Download MoveIt Task Constructor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Move into your colcon workspace and pull the MoveIt Task Constructor source: ::

    cd ~/ws_moveit2/src
    git pull git@github.com:ros-planning/moveit_task_constructor.git -b ros2
    vcs import < moveit_task_constructor/.repos

Create a new package
^^^^^^^^^^^^^^^^^^^^

Create a new package with the following command: ::

    ros2 pkg create --build-type ament_cmake --node-name mtc_tutorial mtc_tutorial

This will create a new folder called ``mtc_tutorial`` with a hello world example in ``src/mtc_node``. Next, add the dependencies to ``package.xml``. It should look similar to this: ::

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>mtc_tutorial</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="youremail@domain.com">user</maintainer>
    <license>TODO: License declaration</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>moveit_task_constructor_core</depend>
    <depend>rclcpp</depend>

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
    </package>

Also, add the dependencies to ``CMakeLists.txt``. The file should look similar to this: ::

    cmake_minimum_required(VERSION 3.8)
    project(mtc_tutorial)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(moveit_task_constructor_core REQUIRED)
    find_package(rclcpp REQUIRED)
    # uncomment the following section in order to fill in
    # further dependencies manually.
    # find_package(<dependency> REQUIRED)

    add_executable(mtc_tutorial src/mtc_tutorial.cpp)
    ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
    target_include_directories(mtc_tutorial PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

    install(TARGETS mtc_tutorial
    DESTINATION lib/${PROJECT_NAME})

    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()


Build and run your new package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build and source your colcon workspace. ::

    cd ~/ws_moveit2
    colcon build --mixin release
    source ~/ws_moveit2/install/setup.bash

You can run the hello world example in the new package like this: ::

    ros2 run mtc_tutorial mtc_tutorial

You should see the following text: ::

    hello world mtc_tutorial package

Setting up with MoveIt Task Constructor
---------------------------------------

This section walks through the code required to build a minimal task using MoveIt Task Constructor.

The code
^^^^^^^^

Open ``mtc_tutorial.cpp`` in your editor of choice, and paste in the following code. *TODO: Check if this compiles and behaves as expected*

.. code-block:: c++

    #include <rclcpp/rclcpp.hpp>
    #include <moveit/planning_scene/planning_scene.h>
    #include <moveit/planning_scene_interface/planning_scene_interface.h>
    #include <moveit/task_constructor/task.h>
    #include <moveit/task_constructor/solvers.h>
    #include <moveit/task_constructor/stages.h>
    #include <std_srvs/srv/trigger.hpp>

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

    class MTCTaskNode
    {
    public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

    private:
    // Compose an MTC task from a series of stages.
    moveit::task_constructor::Task createTask();


    private:
    rclcpp::Node::SharedPtr node_;
    };

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
    return node_->get_node_base_interface();
    }

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {

    }

    void MTCTaskNode::setupPlanningScene()
    {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1, 0.02 };

    geometry_msgs::msg::Pose pose;
    pose.position.z += 0.2;
    pose.position.x += 0.6;
    object.primitive_poses.push_back(pose);

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
    }


    void MTCTaskNode::doTask()
    {
    moveit::task_constructor::Task task = createTask();

    try
    {
        task.init();
    }
    catch (moveit::task_constructor::InitStageException& e) {

        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!task.plan())
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
    }

    moveit::task_constructor::Task MTCTaskNode::createTask()
    {
    moveit::task_constructor::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "panda_arm";
    const auto& eef_name = "hand";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "panda_link8";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", eef_name);
    task.setProperty("hand", hand_group_name);
    task.setProperty("hand_grasping_frame", hand_frame);
    task.setProperty("ik_frame", hand_frame);

    moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);

    auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScaling(1.0);
    cartesian_planner->setMaxAccelerationScaling(1.0);
    cartesian_planner->setStepSize(.01);

    auto scene{ std::make_shared<planning_scene::PlanningScene>(task.getRobotModel()) };
    auto& robot_state{ scene->getCurrentStateNonConst() };

    return task;
    }

    int main(int argc, char** argv)
    {

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    executor.cancel();
    spin_thread->join();

    rclcpp::shutdown();
    return 0;
    }


Code Breakdown
^^^^^^^^^^^^^^

The top of the code includes the ROS and MoveIt Libraries that this package uses.
 * ``rclcpp/rclcpp.hpp`` includes core ROS2 functionality
 * ``moveit/planning_scene/planning_scene.h`` and ``moveit/planning_scene_interface/planning_scene_interface.h`` includes functionality to interface with the robot model and collision objects
 * ``moveit/task_constructor/task.h`, ``moveit/task_constructor/solvers.h``, and ``moveit/task_constructor/stages.h`` include different components of MoveIt Task Constructor that are used in the example
 * ``std_srvs/srv/trigger.hpp`` is used to provide functionality to make service calls 
  
.. code-block:: c++

    #include <rclcpp/rclcpp.hpp>
    #include <moveit/planning_scene/planning_scene.h>
    #include <moveit/planning_scene_interface/planning_scene_interface.h>
    #include <moveit/task_constructor/task.h>
    #include <moveit/task_constructor/solvers.h>
    #include <moveit/task_constructor/stages.h>
    #include <std_srvs/srv/trigger.hpp>

The next line gets a logger for your new node.

.. code-block:: c++

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

We start by defining a class that will contain the main MoveIt Task Constructor functionality. We will explore each function individually below.

.. code-block:: c++

    class MTCTaskNode
    {
    public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

    private:
    // Compose an MTC task from a series of stages.
    moveit::task_constructor::Task createTask();


    private:
    rclcpp::Node::SharedPtr node_;
    };

These lines define a getter function to get the node base interface, which will be used for the executor later.

.. code-block:: c++

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
    return node_->get_node_base_interface();
    }

These next lines initialize the node with specified options. 

.. code-block:: c++

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {

    }

This class method is used to set up the planning scene that is used in the example. It creates a cylinder with dimensions specified by ``object.primitives[0].dimensions`` and position specified by ``pose.position.z`` and ``pose.position.x``. After you build and run your package once, try changing these numbers to resize and move the cylinder around. If you move the cylinder out of the robot's reach, planning will fail.

.. code-block:: c++

    void MTCTaskNode::setupPlanningScene()
    {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1, 0.02 };

    geometry_msgs::msg::Pose pose;
    pose.position.z += 0.2;
    pose.position.x += 0.6;
    object.pose.push_back(pose);

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
    }

This function interfaces with the MoveIt Task Constructor task object. It first creates a task, which includes setting some properties and adding stages. This will be discussed further in the ``createTask`` function definition. Next, ``task.init()`` initializes the task, ``task.plan()`` generates a plan, and ``task.execute()`` executes the plan.

.. code-block:: c++

    void MTCTaskNode::doTask()
    {
    moveit::task_constructor::Task task = createTask();

    try
    {
        task.init();
    }
    catch (moveit::task_constructor::InitStageException& e) {

        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!task.plan())
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }

    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
    }

As mentioned above, this function creates a MoveIt Task Constructor object and sets some initial properties. In this case, we set the task name to "demo_task", load the robot model, define the names of some useful frames, and set those frame names as properties of the task with ``task.setProperty(property_name, value)``.

.. code-block:: c++

    moveit::task_constructor::Task MTCTaskNode::createTask()
    {
    moveit::task_constructor::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "panda_arm";
    const auto& eef_name = "hand";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "panda_link8";

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", eef_name);
    task.setProperty("hand", hand_group_name);
    task.setProperty("hand_grasping_frame", hand_frame);
    task.setProperty("ik_frame", hand_frame);

Planners

.. code-block:: c++

    moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);

    auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScaling(1.0);
    cartesian_planner->setMaxAccelerationScaling(1.0);
    cartesian_planner->setStepSize(.01);

    auto scene{ std::make_shared<planning_scene::PlanningScene>(task.getRobotModel()) };
    auto& robot_state{ scene->getCurrentStateNonConst() };

    return task;
    }

main 

.. code-block:: c++

    int main(int argc, char** argv)
    {

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    executor.cancel();
    spin_thread->join();

    rclcpp::shutdown();
    return 0;
    }

Adding Stages
-------------

TBD

Visualizing with RViz
---------------------

To use MoveIt Task Constructor, you will need to have a robot setup with MoveIt configurations and launch a number of nodes. For this tutorial, we will use an :mtc_codedir:`example launch file <demo/launch/demo.launch.py>` for the panda robot. ::

    ros2 launch moveit_task_constructor_demo demo.launch.py

Rviz should load with the panda in the main view. The Motion Planning Tasks pane should be open in the bottom left, with nothing in it. Your MTC task will show up in this pane once you launch the ``mtc_tutorial`` node. To run your MoveIt Task Constructor node: ::

    ros2 launch mtc_tutorial mtc_tutorial.launch.py

The task with each comprising stage is shown in the Motion Planning Tasks pane. Click on a stage and additional information about the stage will show up to the right. The right pane shows different solutions as well as their associated costs. Depending on the stage type and the robot configuration, there may only be one solution shown. 

Click a solution to see an animation of the robot following the plan for that stage. Click the "Exec" button in the upper-right portion of the pane to execute the motion.
