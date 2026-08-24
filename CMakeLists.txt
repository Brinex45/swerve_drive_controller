cmake_minimum_required(VERSION 3.16)
project(swerve_drive_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rcpputils
  tf2
  tf2_geometry_msgs
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(swerve_drive_controller_parameters
  src/swerve_drive_controller_parameters.yaml
)

add_library(swerve_drive_controller SHARED
  src/swerve_drive_controller.cpp
)
target_compile_features(swerve_drive_controller PUBLIC cxx_std_17)
target_include_directories(swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/swerve_drive_controller>
)
target_link_libraries(swerve_drive_controller PUBLIC
  swerve_drive_controller_parameters
)
ament_target_dependencies(swerve_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# --- Piece 3a: pluginlib export ---
# Registers this package's controller_interface plugin description
# (swerve_drive_controller.xml, written in Piece 4) in the ament index,
# so controller_manager can discover and dlopen() the library by the
# plugin name string used in a robot's controller YAML config.
pluginlib_export_plugin_description_file(controller_interface swerve_drive_controller.xml)

# --- Piece 3b: install rules ---

# 1. Install the built libraries (the controller itself, plus the
#    auto-generated parameter-listener library) into lib/, and register
#    them under the export name used below so downstream packages can
#    link against them via ament_export_targets.
install(
  TARGETS swerve_drive_controller swerve_drive_controller_parameters
  EXPORT export_swerve_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 2. Install public headers so `#include <swerve_drive_controller/...>`
#    resolves for anything building against this package.
install(
  DIRECTORY include/
  DESTINATION include/swerve_drive_controller
)

# Note: the plugin XML itself does NOT need an explicit install() call —
# pluginlib_export_plugin_description_file() above already handles
# installing it to share/swerve_drive_controller/ and registering it
# in the ament index. Adding a separate install() for it would be
# redundant.

# --- Piece 3c: test build rules ---
if(BUILD_TESTING)
  # 1. Lint / static checks — near-mandatory for upstream ros2_controllers
  #    review. No test code required; these run copyright, style, and
  #    static-analysis checks against the whole package automatically.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # 2. Unit/integration test target.
  #    test_swerve_drive_controller.cpp exercises the SwerveDriveController
  #    class directly (on_init/on_configure/on_activate/update) AND, via
  #    ros2_control_test_assets mock hardware, loads the plugin through
  #    the real controller_manager + pluginlib discovery path — this is
  #    what actually proves the CMake/XML plumbing (Pieces 3a/4) works
  #    end-to-end, not just that the C++ math is correct.
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_swerve_drive_controller
    test/test_swerve_drive_controller.cpp
  )
  target_link_libraries(test_swerve_drive_controller
    swerve_drive_controller
  )
  ament_target_dependencies(test_swerve_drive_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

# 3. Export the targets and dependencies so `find_package(swerve_drive_controller)`
#    works correctly for any package that later depends on this one
#    (e.g. your own test package, or upstream integration).
ament_export_targets(export_swerve_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)

ament_package()