# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AeroStream is a 1D flight control learning platform combining a C++17 physics/control core with a Python Streamlit Ground Control Station (GCS). The C++ binary is invoked as a subprocess by the Python dashboard, which reads the resulting CSV telemetry for visualization, metrics, and noise sensitivity analysis.

## Build & Test Commands

**Build C++ core:**
```bash
mkdir -p build && cd build && cmake .. && cmake --build .
```

**Run unit tests (13 tests across 3 suites):**
```bash
cd build && ./unit_tests
```

**Run a specific test** (GoogleTest filter):
```bash
cd build && ./unit_tests --gtest_filter="MockSensorTest.NoiseMeanIsNearZero"
```

**Run the Streamlit GCS:**
```bash
cd scripts && pip install -r requirements.txt && streamlit run app.py
```

**Full pipeline (build + test + simulate + visualize):**
```bash
./run_all.sh
```

**Run simulation manually (8 args, noise_sigma optional — defaults to 0.5):**
```bash
cd build && ./flight_controller <Kp> <Ki> <Kd> <steps> <target1> <target2> <switch_step> [noise_sigma]
# Example: ./flight_controller 0.6 0.01 0.05 1000 50.0 100.0 500 0.5
```

**Debug build with AddressSanitizer + UBSan:**
```bash
mkdir -p build-san && cd build-san && cmake .. -DCMAKE_BUILD_TYPE=Debug && cmake --build .
cd build-san && ASAN_OPTIONS=detect_leaks=1 ./unit_tests
```

## Architecture

### C++ Core (`src/`, `include/`, `tests/`)

All C++ classes live in `namespace aerostream`.

- **`include/IController.hpp`** — Abstract controller interface (`calculate`, `reset`). The extension point for adding new control strategies. `PID` inherits from it.
- **`include/ISensor.hpp`** — Abstract sensor interface (`init`, `readValue`). `MockSensor` inherits from it.
- **`src/main.cpp`** — Simulation entry point; accepts up to 8 CLI args (7th: switch_step, 8th: noise_sigma). Runs a 10 Hz physics loop (dt = 0.1 s), outputs `build/telemetry.csv` with columns: `Time, Target, Actual, Velocity, Output`.
- **`src/core/PID.cpp`** — PID controller; output clamped to [0.0, 50.0]; anti-windup via conditional integration; first-order derivative filter with coefficient N (default 10.0). Constructor: `PID(kp, ki, kd, dt, max_out, min_out, N=10.0)`.
- **`src/simulation/PhysicsEngine.cpp`** — 1D Newtonian dynamics: gravity (−9.81 m/s²), quadratic drag, ground collision clamp. `setState(position, velocity)` allows direct state initialization (used in tests).
- **`src/simulation/MockSensor.cpp`** — Implements `ISensor`. Gaussian noise via `std::normal_distribution<double>(0, σ)` seeded from `std::random_device`. Use `setValue(v)` to sync to true physics position, then `readValue()` for the noisy measurement.

Build system is CMake 3.10+, C++17, GoogleTest v1.15.2 (pinned). clang-tidy runs during compilation with modernize/readability checks. Debug builds enable ASan + UBSan.

### Python GCS (`scripts/app.py`)

- **Path resolution**: `BUILD_DIR = ROOT_DIR/build`, `EXE_PATH = BUILD_DIR/flight_controller`, `CSV_PATH = BUILD_DIR/telemetry.csv`
- **Two tabs**: "Mission Simulation" (existing flight sim + Twiddle optimizer) and "Noise Analysis" (σ sweep)
- **Simulation flow**: UI parameters → subprocess call to C++ binary (8 args) → read CSV → compute metrics → render Plotly chart
- **Metrics**: Settling time (±2% band), overshoot %, RMSE computed in Python from CSV
- **Noise Analysis tab**: runs 5 simulations at σ = [0.01, 0.1, 0.5, 1.0, 2.0] m, plots RMSE bar chart + overlaid time-series
- **AI Auto-Tuner**: Coordinate Descent (Twiddle); up to 30 iterations, converges when `sum(dp) < 0.005`; always runs at σ = 0.5 m

### CI/CD

GitHub Actions (`.github/workflows/cpp-build.yml`) runs three jobs on push/PR:
1. `build-and-test` — Release build + 13 unit tests via ctest
2. `lint` — installs clang-tidy, builds with `CMAKE_EXPORT_COMPILE_COMMANDS=ON`
3. `sanitizer-tests` — Debug build + unit tests under `ASAN_OPTIONS=detect_leaks=1`

## Key Design Constraints

- The C++ binary and Python dashboard share state only through `build/telemetry.csv`.
- `dt = 0.1 s` (10 Hz) is hardcoded in both the physics simulation and the Streamlit metrics calculations; changing it requires updating both.
- Default physics: `mass = 1 kg`, `drag_coeff = 0.5`, `cross_sectional_area = 0.1 m²`.
- PID `min_output = 0.0`: the drone cannot apply negative thrust (can only coast downward, not actively descend).
- The `IController` interface (`include/IController.hpp`) is the extension point for new controllers; the `ISensor` interface (`include/ISensor.hpp`) is the extension point for new sensor types.
