cmake_minimum_required(VERSION 3.8)
project(leader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)


# Build Nodes

# route_manager node
add_executable(route_manager src/route_manager.cpp)
ament_target_dependencies(route_manager rclcpp std_msgs geometry_msgs px4_msgs)
install(TARGETS route_manager DESTINATION lib/${PROJECT_NAME})

target_include_directories(route_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(route_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# leader node
add_executable(leader src/leader.cpp)
ament_target_dependencies(leader rclcpp std_msgs geometry_msgs px4_msgs)
install(TARGETS leader DESTINATION lib/${PROJECT_NAME})

target_include_directories(leader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(leader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# INSTALL

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime geometry_msgs px4_msgs)

ament_export_include_directories(include)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


# TESTING

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()