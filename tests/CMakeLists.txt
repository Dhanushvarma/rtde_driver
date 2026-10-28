# The gripper against a fake Robotiq URCap socket on localhost
add_executable(test_gripper test_gripper.cpp)
target_link_libraries(test_gripper PRIVATE rtde_gripper)
add_test(NAME test_gripper COMMAND test_gripper)
