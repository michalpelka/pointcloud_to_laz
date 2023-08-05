cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_laz)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

message("LASzip library")
# LASzip library
add_subdirectory(3rd/LASzip)
include_directories(3rd/LASzip/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pointcloud_to_laz src/pointcloud_to_laz.cpp src/save_laz.cpp)
target_include_directories(pointcloud_to_laz PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(pointcloud_to_laz PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(pointcloud_to_laz rclcpp sensor_msgs)
target_link_libraries(pointcloud_to_laz laszip)
install(TARGETS pointcloud_to_laz
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
