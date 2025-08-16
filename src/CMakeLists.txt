cmake_minimum_required(VERSION 3.0.2)
project(robotic_arm_workspace)

find_package(catkin REQUIRED COMPONENTS
  rospy
  trajectory_msgs
  sensor_msgs
  std_msgs
)

# ───────────────────────────────────────────────────────────
# Build Pinocchio as an external project with MeshCat visualize
include(ExternalProject)
ExternalProject_Add(
  pinocchio_ext
  GIT_REPOSITORY    https://github.com/stack-of-tasks/pinocchio.git
  GIT_TAG           master
  PREFIX            ${CMAKE_BINARY_DIR}/pinocchio_ext
  SOURCE_DIR        ${CMAKE_SOURCE_DIR}/pinocchio_ext_src
  BINARY_DIR        ${CMAKE_BINARY_DIR}/pinocchio_ext_build
  INSTALL_DIR       ${CATKIN_DEVEL_PREFIX}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_PYBINDINGS=ON
    -DBUILD_PYTHON_VISUALIZE=ON
    -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
)
ExternalProject_Get_Property(pinocchio_ext install_dir)
list(APPEND CMAKE_PREFIX_PATH "${install_dir}")
list(APPEND PYTHONPATH      "${install_dir}/lib/python3.${PYTHON_VERSION_MAJOR}/site-packages")
add_dependencies(${catkin_LIBRARIES} pinocchio_ext)
# ───────────────────────────────────────────────────────────

catkin_package(
  CATKIN_DEPENDS rospy trajectory_msgs sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${install_dir}/include
)

## Declare your packages here (rosbuild style calls are not needed)
