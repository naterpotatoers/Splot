cmake_minimum_required(VERSION 3.8)
project(quadruped_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(servo_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Set up quadruped_kinematics_lib library
add_library(quadruped_kinematics_lib SHARED
            src/quadruped_kinematics_lib.cpp
            src/kinematics_lib.cpp)
ament_target_dependencies(quadruped_kinematics_lib servo_interfaces)
target_include_directories(quadruped_kinematics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quadruped_kinematics_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_targets(quadruped_kinematics_lib HAS_LIBRARY_TARGET)

install(DIRECTORY include/quadruped_kinematics
  DESTINATION include)

install(TARGETS quadruped_kinematics_lib
  EXPORT quadruped_kinematics_lib
  DESTINATION lib/${PROJECT_NAME})

# Set up package executable
add_executable(quadruped_kinematics
               src/quadruped_kinematics.cpp)
ament_target_dependencies(quadruped_kinematics rclcpp servo_interfaces)
target_link_libraries(quadruped_kinematics quadruped_kinematics_lib)
target_include_directories(quadruped_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quadruped_kinematics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS quadruped_kinematics
  DESTINATION lib/${PROJECT_NAME})

# Set up inverse kinematics test executable
add_executable(kinematics_lib_test
               test/kinematics_lib_test.cpp
               src/kinematics_lib.cpp)
target_include_directories(kinematics_lib_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinematics_lib_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

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

ament_package()
