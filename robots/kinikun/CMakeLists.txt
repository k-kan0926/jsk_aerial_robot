cmake_minimum_required(VERSION 3.0.2)
project(kinikun)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_model
  aerial_robot_msgs
  diagnostic_msgs
  pluginlib
  roscpp
  dynamic_reconfigure
  std_msgs
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

# acados が /home/kan/acados にある前提
# 環境変数 ACADOS_SOURCE_DIR があればそっちを優先
set(ACADOS_ROOT $ENV{ACADOS_SOURCE_DIR})
if(NOT ACADOS_ROOT)
  set(ACADOS_ROOT /home/keiichiro/acados)
endif()

# 生成コードがある場所（あなたのツリーに合わせる）
set(KINIKUN_ACADOS_GEN_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/models/narx_disc/c_generated_code
)

# ここで .c を全部拾うけど、main_*.c は後で除外する
file(GLOB KINIKUN_ACADOS_C_SOURCES
  ${KINIKUN_ACADOS_GEN_DIR}/*.c
  ${KINIKUN_ACADOS_GEN_DIR}/*/*.c
)

# --- ① acados が吐いたサンプル main はコンパイルしない ---
list(REMOVE_ITEM KINIKUN_ACADOS_C_SOURCES
  ${KINIKUN_ACADOS_GEN_DIR}/main_narx_disc.c
)

# --- ② ライブラリとしてまとめる ---
add_library(kinikun_acados_generated STATIC
  ${KINIKUN_ACADOS_C_SOURCES}
)

# acados のヘッダを見えるようにする
# acados を "make install" してれば ${ACADOS_ROOT}/include に入ってます
target_include_directories(kinikun_acados_generated PUBLIC
  ${ACADOS_ROOT}/include
  ${ACADOS_ROOT}/include/blasfeo/include
  ${ACADOS_ROOT}/include/hpipm/include
  ${KINIKUN_ACADOS_GEN_DIR}
  ${KINIKUN_ACADOS_GEN_DIR}/narx_disc_model
  ${KINIKUN_ACADOS_GEN_DIR}/narx_disc_cost
)

# acados のコアライブラリにリンク
# (インストールしてあるものに合わせてパスを変えてください)
target_link_directories(kinikun_acados_generated PUBLIC
  ${ACADOS_ROOT}/lib
)
target_link_libraries(kinikun_acados_generated PUBLIC
  acados
  hpipm
  blasfeo
)

# -----------------------------------------------------------
# catkin 設定
# -----------------------------------------------------------

generate_dynamic_reconfigure_options(
  config/PID.cfg
  config/Angle_PID.cfg
  config/NarxController.cfg
)

catkin_package(
  CATKIN_DEPENDS
    aerial_robot_control
    aerial_robot_model
    dynamic_reconfigure
    aerial_robot_msgs
    pluginlib
    roscpp
    diagnostic_msgs
    std_msgs
    geometry_msgs
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES 
    kinikun_acados_generated
    kinikun_controller
)

# 生成したacadosコードの場所をターゲットにも見せる
set(KINIKUN_ACADOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/models/narx_disc)
set(KINIKUN_ACADOS_GEN_DIR ${KINIKUN_ACADOS_DIR}/c_generated_code)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${KINIKUN_ACADOS_DIR}
  ${KINIKUN_ACADOS_GEN_DIR}
)

add_library(kinikun_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(kinikun_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(kinikun_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

add_library(kinikun_controller src/control/kinikun_controller.cpp)
target_link_libraries(kinikun_controller kinikun_sensor_pluginlib ${catkin_LIBRARIES})


# あなたのノード (mpa_acados_node.cpp とか) をここでビルド
add_executable(mpa_acados_node
  src/mpa_acados_node.cpp
  src/narx_acados_solver.cpp   # あるなら
)
target_link_libraries(mpa_acados_node
  ${catkin_LIBRARIES}
  kinikun_acados_generated
)

install(TARGETS kinikun_sensor_pluginlib kinikun_controller
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY config plugins launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)