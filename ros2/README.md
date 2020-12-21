

## Creating a ROS workspace

ROS 2 requires a workspace where all your packages are placed. 
Compilation is always done at the root foulder of the workspace   

```shell
$ mkdir -p ~/workspace_name/src
$ cd ~/workspace_name/
$ colcon build  # --> initializes the workspace
```

Other useful arguments for colcon build:
* `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
* `--symlink-install` saves you from having to rebuild every time you tweak python scripts
* `--event-handlers console_direct+` shows console output while building (can otherwise be found in the log directory)

## Creating a package

A ROS package is a bundle of related ROS nodes / functions.
For example you can have a package for camera nodes, microphone nodes, etc. ...

```shell
$ cd ~/workspace_name/src  #--> packages are stored in the src foulder
$ ros2 pkg create --build-type ament_python <package_name> # Python build package
$ ros2 pkg create --build-type ament_cmake <package_name>  # CMake build package

```

After creating a package you can customize the information about it in package.xml.

## Build a package

```shell
colcon build # Build all packages
colcon build --packages-select my_package # Build a specific package
```

After building the package, source it with:
```shell
cd ~/workspace_name/
. install/setup.bash
```

It’s good practice to run rosdep in the root of your workspace (dev_ws) to check for missing dependencies before building
```shell
rosdep install -i --from-path src --rosdistro eloquent -y
```


## Add workspace to your environment

To announce your created packages and nodes to ROS infrastructure and your shell (so that you can use autocompletion with <tab>), 
you have to run the following command in every new terminal.

```shell
source /opt/ros/eloquent/setup.bash
```

If you don’t want to have to source the setup file every time you open a new shell, then you can add the command to your shell startup script:
```shell
echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
```

# Run ROS

```shell
ros2 run my_package my_node
```

# Utils

```shell
ros2 node list                # --> will show you the names of all running nodes
ros2 node info <node_name>    # --> access more information about a node
ros2 topic list [-t]          # --> return a list of all the topics currently active in the system, -t appends topic type 
ros2 topic echo <topic_name>  # --> To see the data being published on a topic
ros2 topic info <topic_name>  # --> info about topic with amount of subs and pubs


ros2 interface show <type>.msg        # --> shows the structure of message data
ros2 interface show <type_name>.srv   # --> structure of the input arguments for service

ros2 service list                 # --> return a list of all the services currently active in the system
ros2 service list -t              # --> return a list of all the services with their types
ros2 service type <service_name>  # --> To find out the type of a service
ros2 service find <type_name>     # --> find all the services of a specific type


```

Do things from shell:
```shell
## Messages ##
ros2 topic pub <topic_name> <msg_type> '<args>'
# e.g.
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

## Services ##
ros2 service call <service_name> <service_type> <arguments>
# e.g.
ros2 service call /clear std_srvs/srv/Empty

```

# Write a Python Node

If you have written a python node, you have to add an entry point in the `setup.py`of the package.
```py
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
                '<nodeName> = <packageName>.<pythonScriptName>:<function>
        ],
},
```

# Write a C++ Node


# Python Node and C++ Node in same package

Should actually work, not tested yet. There are some examples from ROS-Forums.

Create Package as C++ package. The python nodes can be defined in the CMakeList.txt.

Example from https://github.com/tanyouliang95/MySandBox/blob/master/ros2_payload/CMakeLists.txt

``` makefile
cmake_minimum_required(VERSION 3.5)
project(ros2_payload)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -lwiringPi -lwiringPiDev")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # Python dependency
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)
# include_directories(include ${WIRINGPI_INCLUDE_DIRS})

ament_python_install_package(scripts/)  # Python install

## C++ Nodes
add_executable(slide_door_wrap src/SlideDoor.cpp)
ament_target_dependencies(slide_door_wrap rclcpp std_msgs)

add_executable(mock_publisher src/MockPublisher.cpp)
ament_target_dependencies(mock_publisher rclcpp std_msgs)

## Install C++ Nodes
install(TARGETS
  mock_publisher
  slide_door_wrap
  DESTINATION lib/${PROJECT_NAME}
)

## Install Python Nodes
install(PROGRAMS
  scripts/PayloadController.py
  DESTINATION bin
)

ament_package()
```

# Create own messages

You can create your own message types for topic messages.
Following the tutorial on: https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/

Custom messages **MUST** be defined in a C++-Package.

Steps:
1.) Create directory "msg" in ~/workspace/src/cppMessagePackage/
2.) Create your custom message files
Example:
MyMessage.msg in ~/workspace/src/cppMessagePackage/msg:
```
  float32 x
  float32 y
  float32 width
  float32 height
  string name
```

3.) Edit ~/workspace/src/cppMessagePackage/package.xml 
Because the interfaces rely on rosidl_default_generators for generating language-specific code, you need to declare a dependency on it. Add the following lines to package.xml
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

4.) Edit ~/workspace/src/cppMessagePackage/CMakeLists.txt
To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines to CMakeLists.txt:
```Makefile
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"     # For messages
  "srv/AddThreeInts.srv"  # For services
  ...
 )
```

5.) Build the package to generate the messages.
```shell
colcon build --packages-select cppMessagePackage
```


## Use message

After you have created and installed your custom message
you can import your custom messages in any script (in the same package OR in packages that have your package as dependency):

Python:
```python
from cppMessagePackage.msg import MyMessage

msg = MyMassage()
msg.x = float(1)
msg.y = float(2)
...
```

In order to use the messages, you have to define the dependencies in your CMakeList.txt (Only for C++) and the package.xml

CMakeLists.txt add the following lines (C++ only):
```makefile
find_package(cppMessagePackage REQUIRED)  # Find dependency package

...

ament_target_dependencies(talker rclcpp dep1 cppMessagePackage)

```

package.xml add the following line:

```xml
<!-- For C++ -->
<depend>tutorial_interfaces</depend> 
<!-- For Python -->
<exec_depend>tutorial_interfaces</exec_depend>
```


# Publisher and subscriber
Python:
https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

C++:
https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/


# Sources
https://index.ros.org/doc/ros2/Tutorials/



# Errors

## not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH
```
CMake Error at CMakeLists.txt:19 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake

  Add the installation prefix of "ament_cmake" to CMAKE_PREFIX_PATH or set
  "ament_cmake_DIR" to a directory containing one of the above files.  If
  "ament_cmake" provides a separate development package or SDK, be sure it
  has been installed.


---
Failed   <<< cpp_pubsub [0.05s, exited with code 1]
```

Solution: You have forgot to add the ROS 2 workspace to your environment. Run in your shell:
`source /opt/ros/eloquent/setup.bash`


## Subscription of nodes not working

Subscriptions are only processed, if you called the rclpy.spin(node) function on your node.

Be aware, that (currently) there are problems processing multiple nodes in the same script (e.g. calling multiple times spin()).
Thus try to avoid the use of multiple nodes, but better use on main node in your node and create all subscriptions on this node (e.g. see MotorControl.py)