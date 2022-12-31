#!/bin/bash
set -e

./setup.sh
./build.sh
./test_ros_cpp.sh
