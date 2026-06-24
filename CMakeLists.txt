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
  ament_add_pytest_test(xacro_test test/test_xacro.py)
  ament_add_pytest_test(bridge_configurations_test test/test_bridge_configurations.py)
endif()

ament_package()
