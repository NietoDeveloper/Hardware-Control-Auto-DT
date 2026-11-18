cmake_minimum_required(VERSION 3.20)
project(sdt_vehicle_core VERSION 1.0.0 LANGUAGES CXX
        DESCRIPTION "SoftwareDT real-time vehicle control core (air & ground)")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(SDT_BUILD_TESTS "Build unit / SIL tests" ON)
option(SDT_BUILD_APPS "Build demo applications" ON)
option(SDT_WERROR "Treat warnings as errors" OFF)
set(SDT_SANITIZER "" CACHE STRING "Sanitizers, e.g. 'address,undefined' or 'thread'")

add_library(sdt_warnings INTERFACE)
target_compile_options(sdt_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wnon-virtual-dtor
  -Wold-style-cast -Wcast-align -Wnull-dereference -Wformat=2 -Wundef)
if(SDT_WERROR)
  target_compile_options(sdt_warnings INTERFACE -Werror)
endif()

if(SDT_SANITIZER)
  add_compile_options(-fsanitize=${SDT_SANITIZER} -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=${SDT_SANITIZER})
endif()

find_package(Threads REQUIRED)

add_library(sdt_core STATIC
  src/common/log.cpp
  src/perception/radar_tracker.cpp
  src/safety/geofence.cpp
  src/safety/safety_supervisor.cpp
  src/safety/command_guard.cpp
  src/avoidance/collision_avoidance.cpp
  src/nav/path_follower.cpp
  src/nav/mission_executor.cpp
  src/comms/codec.cpp
  src/comms/transport.cpp
  src/comms/command_gateway.cpp
  src/media/media_router.cpp
  src/runtime/periodic_thread.cpp
  src/runtime/vehicle_controller.cpp
  src/runtime/vehicle_runtime.cpp)
add_library(sdt::core ALIAS sdt_core)
target_include_directories(sdt_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(sdt_core PUBLIC Threads::Threads PRIVATE sdt_warnings)
target_compile_features(sdt_core PUBLIC cxx_std_20)

add_library(sdt_sim STATIC src/sim/sim_world.cpp)
add_library(sdt::sim ALIAS sdt_sim)
target_link_libraries(sdt_sim PUBLIC sdt_core PRIVATE sdt_warnings)

if(SDT_BUILD_APPS)
  add_executable(sil_demo apps/sil_demo/main.cpp)
  target_link_libraries(sil_demo PRIVATE sdt_sim sdt_warnings)
  add_executable(bench_step apps/bench_step/main.cpp)
  target_link_libraries(bench_step PRIVATE sdt_sim sdt_warnings)
endif()

if(SDT_BUILD_TESTS)
  enable_testing()
  file(GLOB SDT_TEST_SOURCES CONFIGURE_DEPENDS tests/*.cpp)
  add_executable(sdt_tests ${SDT_TEST_SOURCES})
  target_link_libraries(sdt_tests PRIVATE sdt_sim sdt_warnings)
  add_test(NAME sdt_tests COMMAND sdt_tests)
  if(SDT_BUILD_APPS)
    add_test(NAME sil_nominal COMMAND sil_demo nominal)
    add_test(NAME sil_link_loss COMMAND sil_demo link-loss)
  endif()
endif()
