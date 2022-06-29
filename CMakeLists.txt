cmake_minimum_required(VERSION 3.0.2)
project(plan2sim)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    std_msgs
    message_generation
    system_controllers
    mj_controller
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  action.msg
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  PerformTask.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  system_controllers
  mj_controller
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
 CATKIN_DEPENDS std_msgs message_runtime actionlib_msgs system_controllers mj_controller
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)
