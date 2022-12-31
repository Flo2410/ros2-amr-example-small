#!/bin/bash
set -e

./setup.sh
./build.sh
./test_ros_cpp.sh
cat build/ros_cpp/Testing/20**/*.xml
