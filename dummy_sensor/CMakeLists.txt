cmake_minimum_required(VERSION 3.8)
project(dummy_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-sensors8 REQUIRED)

include_directories(include/${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED src/DummySensor.cc)
target_link_libraries(${PROJECT_NAME}
  PUBLIC gz-sensors8::gz-sensors8)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}/lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dummy_sensor
  EXPORT dummy_sensorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(dummy_sensor)
ament_export_include_directories(include)

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
