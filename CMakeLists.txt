cmake_minimum_required(VERSION 3.10)
project(AeroStream VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Export compile commands so clang-tidy (and IDEs) can find them
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- clang-tidy (enabled when tool is present; CI installs it explicitly) ---
find_program(CLANG_TIDY_EXE NAMES clang-tidy)
if(CLANG_TIDY_EXE)
  set(CMAKE_CXX_CLANG_TIDY
    "${CLANG_TIDY_EXE};-checks=-*,modernize-use-override,modernize-use-nullptr,modernize-use-default-member-init,readability-braces-around-statements")
endif()

# --- GoogleTest (pinned to v1.15.2) ---
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include_directories(include)

# Shared logic library (used by both flight_controller and unit_tests)
add_library(ControlLogic
  src/core/PID.cpp
  src/simulation/MockSensor.cpp
  src/simulation/PhysicsEngine.cpp
)

add_executable(flight_controller src/main.cpp)
target_link_libraries(flight_controller PRIVATE ControlLogic)

enable_testing()
add_executable(unit_tests
  tests/test_pid.cpp
  tests/test_physics.cpp
  tests/test_sensor.cpp
)
target_link_libraries(unit_tests PRIVATE ControlLogic GTest::gtest_main)

# --- AddressSanitizer + UBSan for Debug builds ---
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  foreach(tgt ControlLogic flight_controller unit_tests)
    target_compile_options(${tgt} PRIVATE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${tgt} PRIVATE
      -fsanitize=address,undefined)
  endforeach()
endif()

include(GoogleTest)
gtest_discover_tests(unit_tests)
