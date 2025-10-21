cmake_minimum_required(VERSION 3.5)
project(eut_robotics_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_package_xml()

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(xacro_test test/test_xacro.py)
endif()

ament_package()
