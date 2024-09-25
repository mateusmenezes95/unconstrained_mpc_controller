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
  eigen3_cmake_module
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

foreach(dependency IN ITEMS ${DEPENDENCIES_FOR_THIS_PACKAGE})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(mpc_matrix_converter
  src/mpc_matrix_converter.cpp
)

target_compile_definitions(mpc_matrix_converter PUBLIC 
  c_std_99 cxx_std_20
)

target_include_directories(mpc_matrix_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mpc_matrix_converter PRIVATE
  fmt
)

ament_target_dependencies(mpc_matrix_converter
  eigen3_cmake_module
)

add_library(unconstrained_mpc_controller
  src/unconstrained_mpc_controller.cpp
)

 # Require C99 and C++20
target_compile_features(unconstrained_mpc_controller PUBLIC
  c_std_99 cxx_std_20
) 

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
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_mpc_matrix_converter
    test/test_mpc_matrix_converter.cpp
  )

  target_include_directories(test_mpc_matrix_converter PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_mpc_matrix_converter
    mpc_matrix_converter
  )
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
