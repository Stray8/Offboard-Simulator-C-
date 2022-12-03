cmake_minimum_required(VERSION 3.0.2)
project(Quarotor_Feedback_Controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES Quarotor_Feedback_Controller
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
            src/QuarotorFeedbackController.cpp
)

add_executable(OFFBOARD src/offboard.cpp)

target_link_libraries(OFFBOARD ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
