cmake_minimum_required(VERSION 3.0.2)
project(kinikun)
option(WITH_ACADOS "Build with acados MPC controller" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  diagnostic_msgs
  pluginlib
  roscpp
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)


if(WITH_ACADOS)

  set(ACADOS_ROOT $ENV{ACADOS_SOURCE_DIR})
  if(NOT ACADOS_ROOT)
    set(ACADOS_ROOT /home/kan/acados)
  endif()

  set(KINIKUN_ACADOS_GEN_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/models/narx_disc/c_generated_code
  )

  file(GLOB KINIKUN_ACADOS_C_SOURCES
    ${KINIKUN_ACADOS_GEN_DIR}/*.c
    ${KINIKUN_ACADOS_GEN_DIR}/*/*.c
  )

  list(REMOVE_ITEM KINIKUN_ACADOS_C_SOURCES
    ${KINIKUN_ACADOS_GEN_DIR}/main_narx_disc.c
  )

  add_library(kinikun_acados_generated STATIC
    ${KINIKUN_ACADOS_C_SOURCES}
  )

  target_include_directories(kinikun_acados_generated PUBLIC
    ${ACADOS_ROOT}/include
    ${ACADOS_ROOT}/include/blasfeo/include
    ${ACADOS_ROOT}/include/hpipm/include
    ${KINIKUN_ACADOS_GEN_DIR}
    ${KINIKUN_ACADOS_GEN_DIR}/narx_disc_model
    ${KINIKUN_ACADOS_GEN_DIR}/narx_disc_cost
  )

  target_link_directories(kinikun_acados_generated PUBLIC
    ${ACADOS_ROOT}/lib
  )
  target_link_libraries(kinikun_acados_generated PUBLIC
    acados
    hpipm
    blasfeo
  )

endif()

generate_dynamic_reconfigure_options(
  config/PID.cfg
  config/Angle_PID.cfg
  config/NarxController.cfg
  config/RelativePID.cfg
)

if(WITH_ACADOS)
  catkin_package(
    CATKIN_DEPENDS
      aerial_robot_control
      aerial_robot_estimation
      aerial_robot_model
      dynamic_reconfigure
    INCLUDE_DIRS include
    LIBRARIES kinikun_acados_generated kinikun_controller_pluginlib
  )
else()
  catkin_package(
    CATKIN_DEPENDS
      aerial_robot_control
      aerial_robot_estimation
      aerial_robot_model
      dynamic_reconfigure
    INCLUDE_DIRS include
    LIBRARIES kinikun_controller_pluginlib
  )
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${KINIKUN_ACADOS_DIR}
  ${KINIKUN_ACADOS_GEN_DIR}
)

if(WITH_ACADOS)
  set(KINIKUN_ACADOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/models/narx_disc)
  set(KINIKUN_ACADOS_GEN_DIR ${KINIKUN_ACADOS_DIR}/c_generated_code)

  add_executable(mpa_acados_node
    src/mpa_acados_node.cpp
    src/narx_acados_solver.cpp
  )
  target_link_libraries(mpa_acados_node
    ${catkin_LIBRARIES}
    kinikun_acados_generated
  )
endif() 

add_library(kinikun_controller_pluginlib
  src/kinikun_tilted_lqi_controller.cpp)
target_link_libraries(kinikun_controller_pluginlib 
${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_dependencies(kinikun_controller_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

install(DIRECTORY config launch scripts plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)