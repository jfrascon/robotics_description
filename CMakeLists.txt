cmake_minimum_required(VERSION 3.5)
project(robotics_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# This package publishes description resources such as Xacro files, meshes,
# and example configuration files. It does not build or install C++ targets.
set(resource_directories
  config
  doc
  meshes
  urdf
)

foreach(resource_directory IN LISTS resource_directories)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_directory}")
    install(
      DIRECTORY ${resource_directory}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(all_imus_macros_test test/test_all_imus_macros.py)
  ament_add_pytest_test(fork_simple_legacy_test test/test_fork_simple_legacy.py)
  ament_add_pytest_test(imu_box_test test/test_imu_box.py)
  ament_add_pytest_test(set_props_plugin_imu_test test/test_set_props_plugin_imu.py)
  ament_add_pytest_test(um7_test test/test_um7.py)
  ament_add_pytest_test(xacro_test test/test_xacro.py)
  ament_add_pytest_test(bridge_configurations_test test/test_bridge_configurations.py)
  ament_add_pytest_test(box_geometry_migrations_test test/test_box_geometry_migrations.py)
  ament_add_pytest_test(cylinder_geometry_migrations_test test/test_cylinder_geometry_migrations.py)
  ament_add_pytest_test(orbbec_gemini335le_test test/test_orbbec_gemini335le.py)
endif()

ament_package()
