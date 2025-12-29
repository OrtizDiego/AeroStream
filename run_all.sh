#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "------------------------------------------"
echo "🚀 AeroStream: Starting Build & Test Suite"
echo "------------------------------------------"

# 1. Clean and Create Build Directory
if [ -d "build" ]; then
    echo "[1/4] Cleaning old build files..."
    rm -rf build
fi
mkdir build
cd build

# 2. Configure and Build C++
echo "[2/4] Configuring and Building C++..."
cmake ..
cmake --build .

# 3. Run C++ Unit Tests
echo "[3/4] Running Unit Tests..."
./unit_tests

# 4. Run the Flight Controller
echo "[4/4] Running Flight Simulation..."
./flight_controller

echo "------------------------------------------"
echo "✅ Build and Simulation Successful!"
echo "Telemetery data generated: build/telemetry.csv"
echo "------------------------------------------"

# 5. Launch Python Visualization
echo "📊 Launching Telemetry Dashboard..."
cd ../scripts
python3 visualize.py