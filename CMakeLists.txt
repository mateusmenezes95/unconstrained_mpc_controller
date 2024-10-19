cmake_minimum_required(VERSION 3.8)
project(unconstrained_mpc_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(DEPENDENCIES_FOR_THIS_PACKAGE
  backward_ros
  controller_interface
  eigen3_cmake_module
  hardware_interface
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

foreach(dependency IN ITEMS ${DEPENDENCIES_FOR_THIS_PACKAGE})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(mpc_matrix_utils
  src/mpc_matrix_converter.cpp
  src/mpc_vectors_manipulator.cpp
)

target_compile_definitions(mpc_matrix_utils PUBLIC 
  c_std_99 cxx_std_20
)

target_include_directories(mpc_matrix_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mpc_matrix_utils PUBLIC
  fmt
)

ament_target_dependencies(mpc_matrix_utils PUBLIC
  eigen3_cmake_module
)

add_library(hw_interfaces_helper INTERFACE)
target_include_directories(hw_interfaces_helper INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(hw_interfaces_helper INTERFACE
  fmt
)

add_library(hw_ifaces_eigen_vec_bridges SHARED
  src/eigen_vector_cmd_interface_bridge.cpp
  src/state_interface_eigen_vector_bridge.cpp
)

target_compile_definitions(hw_ifaces_eigen_vec_bridges PUBLIC 
  c_std_99 cxx_std_20
)

target_include_directories(hw_ifaces_eigen_vec_bridges PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(hw_ifaces_eigen_vec_bridges PUBLIC
  mpc_matrix_utils
  fmt
)

ament_target_dependencies(hw_ifaces_eigen_vec_bridges PUBLIC
  hardware_interface
)

generate_parameter_library(
  unconstrained_mpc_controller_params
  src/unconstrained_mpc_controller_params.yaml
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

target_link_libraries(unconstrained_mpc_controller PUBLIC
  fmt
  hw_ifaces_eigen_vec_bridges
  hw_interfaces_helper
  mpc_matrix_utils
  unconstrained_mpc_controller_params
)

ament_target_dependencies(unconstrained_mpc_controller PUBLIC
  ${DEPENDENCIES_FOR_THIS_PACKAGE}
)

pluginlib_export_plugin_description_file(controller_interface
  plugin_unconstrained_mpc_controller.xml
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    unconstrained_mpc_controller
    unconstrained_mpc_controller_params
    hw_ifaces_eigen_vec_bridges
    hw_interfaces_helper
    mpc_matrix_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/future_refs_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_mpc_matrix_converter
    test/test_mpc_matrix_converter.cpp
  )

  target_include_directories(test_mpc_matrix_converter PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_mpc_matrix_converter
    mpc_matrix_utils
  )

  ament_add_gmock(test_mpc_vectors_manipulator
    test/test_mpc_vectors_manipulator.cpp
  )

  target_include_directories(test_mpc_vectors_manipulator PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_mpc_vectors_manipulator
    mpc_matrix_utils
  )

  ament_add_gmock(test_hw_ifaces_eigen_vec_bridges
    test/test_hw_ifaces_eigen_vec_bridges.cpp
  )

  target_include_directories(test_hw_ifaces_eigen_vec_bridges PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_hw_ifaces_eigen_vec_bridges
    hw_ifaces_eigen_vec_bridges
  )

  ament_target_dependencies(test_hw_ifaces_eigen_vec_bridges
    hardware_interface
  )

  ament_add_gmock(test_hw_interfaces_helper
    test/test_hw_interfaces_helper.cpp
  )
  
  target_include_directories(test_hw_interfaces_helper PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_hw_interfaces_helper
    hw_interfaces_helper
  )

  ament_target_dependencies(test_hw_interfaces_helper
    hardware_interface
  )

  install(
    DIRECTORY test/config
    DESTINATION test
  )

  ament_add_gmock(test_unconstrained_mpc_controller
    test/test_unconstrained_mpc_controller.cpp
  )
  
  target_include_directories(test_unconstrained_mpc_controller PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  
  ament_target_dependencies(test_unconstrained_mpc_controller
    controller_interface
    hardware_interface
    eigen3_cmake_module
    rclcpp
    rclcpp_lifecycle
    std_msgs
  )

  target_link_libraries(test_unconstrained_mpc_controller
    unconstrained_mpc_controller
  )

  add_rostest_with_parameters_gmock(test_load_unconstrained_mpc_controller
    test/test_load_unconstrained_mpc_controller.cpp
    test/config/test_unconstrained_mpc_controller_params.yaml
  )

  ament_target_dependencies(test_load_unconstrained_mpc_controller
    controller_manager
    ros2_control_test_assets
  )

  target_compile_definitions(test_load_unconstrained_mpc_controller PRIVATE
    TEST_DIRECTORY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
endif()

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()
