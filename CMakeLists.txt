cmake_minimum_required(VERSION 3.8)
project(flir_boson_usb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

include_directories(
  include  
  ${OpenCV_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
#  ${PCL_LIBRARY_DIRS}
)

# add boson library
add_library(bosoncamera 
  SHARED
  src/BosonCamera.cpp
)
target_include_directories(bosoncamera
  PUBLIC
  include
)
target_link_libraries(bosoncamera
  ${OpenCV_LIBRARIES}
)

# bounding component
#ament_auto_add_library(flir_boson_usb_component SHARED
#  src/flir_boson_usb_component.cpp 
#)
#target_link_libraries(flir_boson_usb_component
#  bosoncamera
#)
#rclcpp_components_register_nodes(flir_boson_usb_component
#  "flir_boson_component::BosonCamera"
#)
#ament_auto_add_executable(flir_boson_usb_node
#  src/flir_boson_usb_node.cpp
#)

ament_auto_add_executable(BosonLifeCycle
  src/BosonLifeCycle.cpp
)
target_link_libraries(BosonLifeCycle
  bosoncamera
)

# demo program
add_executable(demo src/demo.cpp)
target_link_libraries(demo
  bosoncamera
)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  bosoncamera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_auto_package()
