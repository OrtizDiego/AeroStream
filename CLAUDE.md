# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AeroStream is a drone flight simulation system combining a C++17 physics/control core with a Python Streamlit Ground Control Station (GCS) frontend. The C++ binary is invoked as a subprocess by the Python dashboard, which reads the resulting CSV telemetry for visualization and metrics.

## Build & Test Commands

**Build C++ core:**
```bash
mkdir -p build && cd build && cmake .. && cmake --build .
```

**Run unit tests:**
```bash
cd build && ./unit_tests
```

**Run a specific test** (GoogleTest filter):
```bash
cd build && ./unit_tests --gtest_filter="PIDTest.ZeroError"
```

**Run the Streamlit GCS:**
```bash
cd scripts && pip install -r requirements.txt && streamlit run app.py
```

**Full pipeline (build + test + simulate + visualize):**
```bash
./run_all.sh
# or with Python venv:
./run_all_with_venv.sh
```

**Run simulation manually:**
```bash
cd build && ./flight_controller <Kp> <Ki> <Kd> <steps> <target1> <target2> <switch_step>
# Example: ./flight_controller 0.6 0.01 0.05 1000 50.0 100.0 500
```

## Architecture

### C++ Core (`src/`, `include/`, `tests/`)

- **`src/main.cpp`** — Simulation entry point; accepts 7 CLI args, runs a 1 kHz physics loop, outputs `build/telemetry.csv` with columns: `Time, Target, Actual, Output`
- **`src/core/PID.cpp`** — PID controller with output clamping `[-50.0, 50.0]` and anti-windup; derivative is computed on error (not measurement)
- **`src/simulation/PhysicsEngine.cpp`** — Newtonian dynamics: gravity (`-9.81 m/s²`), drag (`0.5 * rho * v² * Cd * A`), and ground collision (position/velocity clamp to 0)
- **`src/simulation/MockSensor.cpp`** — Implements `ISensor` interface with ±0.5 m Gaussian noise

Build system is CMake 3.10+, C++17, with GoogleTest for unit testing.

### Python GCS (`scripts/app.py`)

The Streamlit dashboard is the primary user interface. Key behaviors:

- **Auto-compilation**: On startup, checks for the C++ binary and rebuilds if missing (handles Streamlit Cloud deployments without pre-built binaries)
- **Path resolution**: Uses absolute paths derived from `__file__`; `BUILD_DIR = ROOT_DIR/build`, `EXE_PATH = BUILD_DIR/flight_controller`, `CSV_PATH = BUILD_DIR/telemetry.csv`
- **Simulation flow**: UI parameters → subprocess call to C++ binary → read CSV → compute metrics → render Plotly chart
- **Metrics**: Settling time (±2% band), overshoot %, and RMSE are calculated in Python from telemetry data
- **AI Auto-Tuner**: Coordinate Descent (Twiddle) algorithm; up to 30 iterations, converges when `sum(dp) < 0.005`; two strategies: "Accuracy" (minimize RMSE) or "Balanced" (minimize RMSE + settling time penalty)
- **Visualization**: Animated Plotly chart with Play/Pause and frame-by-frame DVR replay; CSV export available

### CI/CD

GitHub Actions (`.github/workflows/cpp-build.yml`) runs on push/PR: CMake configure → build → `./unit_tests`. No Python tests are run in CI.

## Key Design Constraints

- The C++ binary and Python dashboard share state only through `build/telemetry.csv` — they never communicate via sockets or pipes.
- `dt = 0.1s` is hardcoded in both the physics simulation and the Streamlit metrics calculations; changing it requires updating both.
- Default physics: `mass=1 kg`, `drag_coeff=0.5`, `cross_sectional_area=0.1 m²`.
- The `ISensor` abstract interface (`include/ISensor.hpp`) is the extension point for adding new sensor types.
