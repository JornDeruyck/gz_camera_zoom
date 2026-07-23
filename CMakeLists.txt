cmake_minimum_required(VERSION 3.10.2)
project(gz_camera_zoom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(CameraZoom SHARED src/CameraZoom.cc)
target_include_directories(CameraZoom PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(CameraZoom PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7)

install(
  TARGETS CameraZoom
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Make libCameraZoom.so discoverable by gz-sim's plugin loader, which only
# searches GZ_SIM_SYSTEM_PLUGIN_PATH (no LD_LIBRARY_PATH fallback) — without
# this hook, sourcing the workspace normally would silently fail to load the
# plugin from any SDF that references it by bare filename.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
