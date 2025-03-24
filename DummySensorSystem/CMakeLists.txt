cmake_minimum_required(VERSION 3.8)
project(DummySensorSystem)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

find_package(dummy_sensor REQUIRED)

if(dummy_sensor_FOUND)
  message(STATUS "dummy_sensor found: ${dummy_sensor_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "dummy_sensor not found!")
endif()

# Include the include directory
include_directories(include/${PROJECT_NAME} ${dummy_sensor_INCLUDE_DIRS})

# Add library
add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cc)

ament_target_dependencies(${PROJECT_NAME}
  "gz-plugin${GZ_PLUGIN_VER}"
  "gz-sim${GZ_SIM_VER}"
  "gz-sensors${GZ_SENSORS_VER}"
  "dummy_sensor"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${dummy_sensor_INCLUDE_DIRS}
)

message(STATUS "Linking libraries: ${PROJECT_NAME} -> dummy_sensor")

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
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

ament_package()
    