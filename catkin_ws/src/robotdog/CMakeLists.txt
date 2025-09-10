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

add_executable(udp_pub_sub src/udp_pub_sub.cpp)
add_dependencies(udp_pub_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(udp_pub_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(state_pub src/state_pub.cpp)
add_dependencies(state_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_pub ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(motor_pose src/motor_pose.cpp)
add_dependencies(motor_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_pose ${EXTRA_LIBS} ${catkin_LIBRARIES})

#add_executable(stand_up src/stand_up.cpp)
#add_dependencies(stand_up ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(stand_up ${EXTRA_LIBS} ${catkin_LIBRARIES})

#add_executable(crouch src/crouch.cpp)
#add_dependencies(crouch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(crouch ${EXTRA_LIBS} ${catkin_LIBRARIES})

#add_executable(sit src/sit.cpp)
#add_dependencies(sit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(sit ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(keyboard_cmd src/keyboard_cmd.cpp)
add_dependencies(keyboard_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_cmd ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(cmd_dispatcher src/cmd_dispatcher.cpp)
add_dependencies(cmd_dispatcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_dispatcher ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(stand_up_srv src/stand_up_srv.cpp)
add_dependencies(stand_up_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stand_up_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(crouch_srv src/crouch_srv.cpp)
add_dependencies(crouch_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(crouch_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(sit_srv src/sit_srv.cpp)
add_dependencies(sit_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sit_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(give_paw_srv src/give_paw_srv.cpp)
add_dependencies(give_paw_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(give_paw_srv ${EXTRA_LIBS} ${catkin_LIBRARIES})


install(TARGETS
        udp_pub_sub
        state_pub
        motor_pose
#        stand_up
#        crouch
#        sit
        keyboard_cmd
        cmd_dispatcher
        stand_up_srv
        crouch_srv
        sit_srv
        give_paw_srv
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})