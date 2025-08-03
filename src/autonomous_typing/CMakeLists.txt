cmake_minimum_required(VERSION 3.0.2)
project(autonomous_typing)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  trajectory_msgs
  sensor_msgs 
  cv_bridge
  std_msgs
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS rospy trajectory_msgs sensor_msgs cv_bridge std_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark Python scripts for installation (only existing files)
catkin_install_python(PROGRAMS
  # Main scripts in src/
  src/autonomous.py
  src/detection_node.py
  src/typing_node.py
  src/typing_node_ik.py
  
  # Scripts in src/redundant_code/
  src/redundant_code/a_bit_of_hardcoded.py
  src/redundant_code/a_bit_of_hardcoded2.py
  src/redundant_code/controller.py
  src/redundant_code/final.py
  src/redundant_code/final0.py
  src/redundant_code/gui.py
  src/redundant_code/helper.py
  src/redundant_code/image_preprocessing.py
  src/redundant_code/keyboard_typer.py
  src/redundant_code/keyboard_typing.py
  src/redundant_code/keyboard_typing2.py
  src/redundant_code/keyboard_typing3.py
  src/redundant_code/kinova_autonom.py
  src/redundant_code/multicontroller.py
  src/redundant_code/renforced_key_detection.py
  src/redundant_code/template_matching.py
  src/redundant_code/yolo_det.py
  src/redundant_code/yolo_key_det.py
  src/redundant_code/yolo_key_typing.py
  src/redundant_code/yolo_seg.py
  src/redundant_code/yolo_seg_save.py
  
  # Scripts in src/tools_to_hardcode_keyboard/
  src/tools_to_hardcode_keyboard/json_tonpy.py
  src/tools_to_hardcode_keyboard/point_marker.py
  src/tools_to_hardcode_keyboard/preprocessing.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files (only if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
  )
endif()

## Install configuration files (only if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.perspective"
  )
endif()

## Install URDF files (only if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
endif()