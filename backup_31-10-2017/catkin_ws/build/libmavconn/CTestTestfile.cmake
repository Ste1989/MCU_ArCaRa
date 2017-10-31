# CMake generated Testfile for 
# Source directory: /home/robot/catkin_ws/src/mavros/libmavconn
# Build directory: /home/robot/catkin_ws/build/libmavconn
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(_ctest_libmavconn_gtest_mavconn-test "/home/robot/catkin_ws/build/libmavconn/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/robot/catkin_ws/build/libmavconn/test_results/libmavconn/gtest-mavconn-test.xml" "--return-code" "/home/robot/catkin_ws/devel/.private/libmavconn/lib/libmavconn/mavconn-test --gtest_output=xml:/home/robot/catkin_ws/build/libmavconn/test_results/libmavconn/gtest-mavconn-test.xml")
SUBDIRS(gtest)
