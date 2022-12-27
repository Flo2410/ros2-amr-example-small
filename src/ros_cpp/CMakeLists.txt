cmake_minimum_required(VERSION 3.14)
project(ros_cpp)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-std=gnu++11 -Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(msg_package REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})
target_link_libraries(${YAML_CPP_LIBRARIES})

ament_export_include_directories(include)
set(dependencies "geometry_msgs" "visualization_msgs" "rclcpp" "msg_package" "rcl_yaml_param_parser" "yaml-cpp")

add_library(cpp_lib src/OrderOptimizerMain.cpp src/OrderOptimizerNode.cpp)
ament_target_dependencies(cpp_lib ${dependencies})
target_link_libraries(cpp_lib yaml-cpp)
add_executable(cpp_node src/OrderOptimizerMain.cpp src/OrderOptimizerNode.cpp)
#ament_target_dependencies(cpp_node ${dependencies})
#target_link_libraries(cpp_node yaml-cpp)
target_link_libraries(cpp_node cpp_lib)

#add_executable(cpp_node src/OrderOptimizerMain.cpp)
#ament_target_dependencies(cpp_node ${dependencies})
#target_link_libraries(cpp_node yaml-cpp)


#add_executable(test1 tests/test1.cpp)
#ament_target_dependencies(test1 ${dependencies})
#target_link_libraries(test1 yaml-cpp)

#add_executable(test2 tests/test2.cpp)
#ament_target_dependencies(test2 ${dependencies})
#target_link_libraries(test2 yaml-cpp)


install(TARGETS
        cpp_node
        DESTINATION lib/${PROJECT_NAME}
        )

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    #find_package(ament_lint_auto REQUIRED)
    #ament_lint_auto_find_test_dependencies()

    include_directories(tests)
    ament_add_gtest(ros_cpp_my_test tests/test2.cpp)
    target_link_libraries(ros_cpp_my_test cpp_lib)


    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
endif ()

ament_package()
