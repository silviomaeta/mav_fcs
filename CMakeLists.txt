cmake_minimum_required(VERSION 2.8.3)
project(mav_fcs)

find_package(catkin REQUIRED
  COMPONENTS
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    tf
    geom_cast
    ca_nav_msgs
    mav_gcs_msgs
    dji_sdk
)

#find_package(Eigen REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(fcs_interface
  src/fcs_interface.cpp
)
add_dependencies(fcs_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcs_interface ${catkin_LIBRARIES})

add_library(path_tracking_control
  src/path_tracking_control.cpp
)
add_dependencies(path_tracking_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracking_control ${catkin_LIBRARIES})

add_library(speed_control
  src/speed_control.cpp
)
add_dependencies(speed_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_control ${catkin_LIBRARIES})

add_library(inspect_ctrl
  src/inspect_ctrl.cpp
)
add_dependencies(inspect_ctrl ${catkin_EXPORTED_TARGETS})
target_link_libraries(inspect_ctrl ${catkin_LIBRARIES})

add_library(fcs_state_machine
  src/fcs_state_machine.cpp
)
add_dependencies(fcs_state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcs_state_machine ${catkin_LIBRARIES})

add_library(copter_interface
  src/copter_interface.cpp
)
add_dependencies(copter_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(copter_interface ${catkin_LIBRARIES})

add_library(copter_interface_simulator
  src/copter_interface_simulator.cpp
)
add_dependencies(copter_interface_simulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(copter_interface_simulator ${catkin_LIBRARIES})

add_library(fcs_processor
  src/fcs_processor.cpp
)
add_dependencies(fcs_processor ${catkin_EXPORTED_TARGETS})
target_link_libraries(fcs_processor ${catkin_LIBRARIES})


add_executable(mav_fcs src/fcs_node.cpp)
add_dependencies(mav_fcs ${catkin_EXPORTED_TARGETS})
target_link_libraries(mav_fcs fcs_processor copter_interface fcs_interface fcs_state_machine inspect_ctrl ${catkin_LIBRARIES})


add_executable(mav_fcs_simulator src/fcs_node.cpp)
add_dependencies(mav_fcs_simulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(mav_fcs_simulator fcs_processor copter_interface_simulator fcs_interface fcs_state_machine inspect_ctrl ${catkin_LIBRARIES})


add_executable(test_speed_control test/test_speed_control.cpp)
add_dependencies(test_speed_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_speed_control speed_control ${catkin_LIBRARIES})


install(TARGETS mav_fcs_simulator mav_fcs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

