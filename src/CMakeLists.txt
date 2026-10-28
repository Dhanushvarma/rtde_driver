# The gripper, also for the tests
add_library(rtde_gripper STATIC Gripper.cpp RobotiqSocket.cpp)
set_target_properties(rtde_gripper PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rtde_gripper PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)
target_link_libraries(rtde_gripper PUBLIC mc_rtc::mc_rtc_utils fmt::fmt Threads::Threads)

add_library(RobotDriverRTDE SHARED RobotDriverRTDE.cpp)

target_include_directories(
  RobotDriverRTDE
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(RobotDriverRTDE
                      PUBLIC unified_robot_interface::robot_driver_api)
target_link_libraries(RobotDriverRTDE PRIVATE ur_client_library::urcl rtde_gripper fmt::fmt)

add_robot_driver(RobotDriverRTDE)

install(
  TARGETS RobotDriverRTDE
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot_interface
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot_interface)
