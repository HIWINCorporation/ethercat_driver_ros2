cmake_minimum_required(VERSION 3.8)
project(ethercat_generic_mdp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(ethercat_generic_slave REQUIRED)
find_package(ethercat_generic_cia402_drive REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)
add_library(ethercat_generic_mdp ${PLUGINS_SRC})
target_compile_features(ethercat_generic_mdp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ethercat_generic_mdp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ethercat_generic_mdp PUBLIC
  ethercat_generic_cia402_drive
  ethercat_generic_slave
  ethercat_interface
  pluginlib
  yaml_cpp_vendor
)

pluginlib_export_plugin_description_file(ethercat_interface ethercat_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ethercat_generic_mdp
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
  include
)
ament_export_libraries(
  ethercat_generic_mdp
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
