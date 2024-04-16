cmake_minimum_required(VERSION 3.8)
project(bms_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(charging_robot_system_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)

ament_auto_find_build_dependencies()

ament_auto_add_executable(bms_interface
  src/bms_interface/bms_interface.cpp
  src/bms_interface/bms_interface_node.cpp
)
target_link_libraries(bms_interface yaml-cpp)

ament_auto_add_executable(inverter_interface
  src/inverter_interface/inverter_interface.cpp
  src/inverter_interface/inverter_interface_node.cpp
)
target_link_libraries(inverter_interface yaml-cpp)

ament_auto_add_executable(mpsbms_interface
  src/mpsbms_interface/mpsbms_interface.cpp
  src/mpsbms_interface/mpsbms_interface_node.cpp
)
target_link_libraries(mpsbms_interface yaml-cpp)

#target_include_directories(mpsbms_interface PRIVATE ${LIBMODBUS_INCLUDE_DIR})
target_link_libraries(mpsbms_interface PkgConfig::LIBMODBUS)

#refactoring
ament_auto_add_executable(ev_charging
  src/ev_charging/ccs_interface.cpp
  src/ev_charging/chademo_interface.cpp
  src/ev_charging/charging_mode_base.cpp
  src/ev_charging/ev_charging_node.cpp
  src/ev_charging/ev_charging.cpp
  src/ev_charging/modbus_interface.cpp
)
target_link_libraries(ev_charging yaml-cpp)

#target_include_directories(mpsbms_interface PRIVATE ${LIBMODBUS_INCLUDE_DIR})
target_link_libraries(ev_charging PkgConfig::LIBMODBUS)

ament_auto_add_executable(test_interface
  src/charge_state_test/test.cpp
)
# ament_target_dependencies(bms_interface charging_robot_system_msgs)

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

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
