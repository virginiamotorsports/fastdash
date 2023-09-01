cmake_minimum_required(VERSION 3.8)
project(fastdash)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# FetchContent_Declare(
#   caplin
#   GIT_REPOSITORY https://github.com/pragmaticlinuxblog/caplin
#   GIT_TAG        1.0.0 
# )
# FetchContent_MakeAvailable(caplin)

# FetchContent_GetProperties(caplin)
#     if(NOT caplin_POPULATED)
#       message("Cloning caplin")
#       FetchContent_Populate(caplin)
#       add_subdirectory(
#         ${caplin_SOURCE_DIR}
#         ${caplin_BINARY_DIR})
#     endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(dash_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(lgpio REQUIRED)


# find_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



include_directories(include)

add_executable(fastdash include/fastdash.hpp src/fastdash.cpp)

# TARGET_LINK_LIBRARIES(fastdash dl -lpigpio)# -pthread  -lcrypt -lm -lpthread)

ament_target_dependencies(fastdash rclcpp rosbag2_cpp std_msgs can_msgs dash_msgs sensor_msgs nav_msgs example_interfaces)


install(TARGETS
fastdash
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
