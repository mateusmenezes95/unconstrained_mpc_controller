cmake_minimum_required(VERSION 3.8)
project(unconstrained_mpc_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(DEPENDENCIES_FOR_THIS_PACKAGE
  backward_ros
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

foreach(dependency IN ITEMS ${DEPENDENCIES_FOR_THIS_PACKAGE})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(unconstrained_mpc_controller
  src/unconstrained_mpc_controller.cpp
)

 # Require C99 and C++20
target_compile_features(unconstrained_mpc_controller PUBLIC c_std_99 cxx_std_20) 

target_include_directories(unconstrained_mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(unconstrained_mpc_controller
  ${DEPENDENCIES_FOR_THIS_PACKAGE}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS unconstrained_mpc_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  unconstrained_mpc_controller
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
