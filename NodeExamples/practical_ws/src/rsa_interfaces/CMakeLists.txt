cmake_minimum_required(VERSION 3.5)
project(rsa_interfaces)

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Id.srv"
  DEPENDENCIES std_msgs # Add packages that above messages depend on
)

#install(
#  DIRECTORY srv test
#  DESTINATION share/${PROJECT_NAME}
#)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

endif()

ament_package()




