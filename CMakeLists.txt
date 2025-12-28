cmake_minimum_required(VERSION 3.10)

# Project Name and Version
project(AeroStream VERSION 1.0.0 LANGUAGES CXX)

# Specify C++ Standard (C++17 is the sweet spot for modern robotics)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories (where header files live)
include_directories(include)

# Gather source files (we will create these shortly)
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Define the executable
add_executable(flight_controller ${SOURCES})

# Future-proofing: We will add testing later
# enable_testing()
