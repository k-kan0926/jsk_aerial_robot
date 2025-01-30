cmake_minimum_required(VERSION 3.0.2)
project(kinikun)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_control
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
  config/PID.cfg
)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
