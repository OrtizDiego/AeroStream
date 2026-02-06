cmake_minimum_required(VERSION 3.10)
project(AeroStream VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- GoogleTest Setup ---
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
# For Windows: prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
# ------------------------

# --- Pybind11 Setup ---
FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v2.10.3.zip
)
FetchContent_MakeAvailable(pybind11)
# ----------------------

include_directories(include)

# Create a library for your logic so both Main and Tests can use it
add_library(ControlLogic src/core/PID.cpp src/simulation/MockSensor.cpp src/simulation/PhysicsEngine.cpp)

# Main Executable
add_executable(flight_controller src/main.cpp)
target_link_libraries(flight_controller PRIVATE ControlLogic)

# Pybind11 Module
pybind11_add_module(aerostream_core src/bindings.cpp)
target_link_libraries(aerostream_core PRIVATE ControlLogic)

# Test Executable
enable_testing()
add_executable(unit_tests tests/test_pid.cpp tests/test_physics.cpp tests/test_feedforward.cpp)
target_link_libraries(unit_tests PRIVATE ControlLogic GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(unit_tests)