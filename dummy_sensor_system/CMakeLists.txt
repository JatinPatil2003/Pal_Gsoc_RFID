cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(DummySensorSystem)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

include_directories(include)

# Fetch the custom sensor example from gz-sensors
# Users won't commonly use this to fetch their sensors. The sensor may be part
# of the system's CMake project, or installed from another project, etc...
include(FetchContent)
FetchContent_Declare(
  sensors_clone
  GIT_REPOSITORY https://github.com/gazebosim/gz-sensors
  GIT_TAG gz-sensors${GZ_SENSORS_VER}
)
FetchContent_Populate(sensors_clone)
add_subdirectory(${sensors_clone_SOURCE_DIR}/examples/custom_sensor ${sensors_clone_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    dummy_sensor
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${sensors_clone_SOURCE_DIR}/examples/custom_sensor)
