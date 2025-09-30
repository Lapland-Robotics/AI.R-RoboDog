cmake_minimum_required(VERSION 3.0.2)
project(robotdog)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosconsole
        rosparam
        roslaunch
        roslib
        geometry_msgs
        unitree_legged_msgs
        unitree_legged_sdk
        unitree_legged_real
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotdog
  CATKIN_DEPENDS unitree_legged_msgs unitree_legged_sdk unitree_legged_real
)

set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${unitree_legged_sdk_INCLUDE_DIRS}
)

add_executable(udp_ros src/udp/udp_ros.cpp)
add_dependencies(udp_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(udp_ros ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(udp_listen src/udp/udp_listen.cpp)
add_dependencies(udp_listen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(udp_listen ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(high_state_logger src/udp/high_state_logger.cpp)
add_dependencies(high_state_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(high_state_logger ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(keyboard_cmd src/cmd/keyboard_cmd.cpp)
add_dependencies(keyboard_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_cmd ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(cmd_dispatcher src/cmd/cmd_dispatcher.cpp)
add_dependencies(cmd_dispatcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_dispatcher ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(stand_up_srv src/movement_servers/stand_up_srv.cpp)
add_dependencies(stand_up_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stand_up_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(crouch_srv src/movement_servers/crouch_srv.cpp)
add_dependencies(crouch_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(crouch_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(sit_srv src/movement_servers/sit_srv.cpp)
add_dependencies(sit_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sit_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(give_paw_srv src/movement_servers/give_paw_srv.cpp)
add_dependencies(give_paw_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(give_paw_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(roll_srv src/movement_servers/roll_srv.cpp)
add_dependencies(roll_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

install(TARGETS
        udp_ros
        udp_listen
        high_state_logger
        keyboard_cmd
        cmd_dispatcher
        stand_up_srv
        crouch_srv
        sit_srv
        give_paw_srv
        roll_srv
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})