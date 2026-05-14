# 🚁 AeroStream: 1D Flight Control Learning Platform

[![Streamlit App](https://static.streamlit.io/badges/streamlit_badge_black_white.svg)](https://aerostream.streamlit.app/)
![CI/CD Status](https://github.com/OrtizDiego/AeroStream/actions/workflows/cpp-build.yml/badge.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)
![C++](https://img.shields.io/badge/C++-17-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-yellow.svg)

A simulation platform for comparing and benchmarking control strategies on a 1D vertical drone dynamics model. Currently implements a PID controller with filtered derivative action. The `IController` abstract interface makes adding new strategies (bang-bang, LQR, MPC) a matter of implementing a single class.

![Simulation screenshot](assets/simulation.png?raw=true)

---

## Architecture

```mermaid
graph LR
    A[Streamlit GCS] -- "Kp, Ki, Kd, σ, Mission" --> B(C++ Simulation)
    B --> C[PhysicsEngine\n1D Newtonian dynamics]
    C -- "true altitude" --> D[MockSensor\nGaussian noise N(0, σ)]
    D -- "measured altitude" --> E[PID Controller\nimplements IController]
    E -- "motor force" --> C
    B -- "telemetry.csv" --> A
    A -- "charts + metrics" --> User
```

### Physics Model

- **Simulation rate:** 10 Hz (dt = 0.1 s), Forward Euler integration
- **Dynamics:** 1D vertical axis — gravity (−9.81 m/s²) + quadratic aerodynamic drag (`0.5 · ρ · v² · Cd · A`)
- **Ground constraint:** position and velocity clamped to 0 on contact
- **Default parameters:** mass = 1 kg, Cd = 0.5, A = 0.1 m²

### Sensor Model

`MockSensor` samples additive Gaussian noise from `std::normal_distribution<double>(0, σ)`,
seeded per-instance via `std::random_device`. σ defaults to 0.5 m and is configurable as
the 8th CLI argument. The sensor is properly wired into the control loop — measured altitude
diverges from true altitude proportionally to σ.

### IController Interface

```cpp
// include/IController.hpp
namespace aerostream {
class IController {
public:
    virtual ~IController() = default;
    virtual double calculate(double setpoint, double pv) = 0;
    virtual void reset() = 0;
};
}
```

Any new controller (LQR gain matrix, MPC solver, bang-bang) implements this interface.
`main.cpp` and the test suite operate against `IController*` — no other changes required.

### PID Controller Details

- Three-term PID with **conditional integration anti-windup** (integral freezes when output saturates in the direction of error)
- **First-order low-pass filtered derivative:** `α = N·dt / (1 + N·dt)`, default N = 10 — prevents derivative kick on step inputs and attenuates high-frequency sensor noise
- Output clamped to [0, 50] N (drone cannot actively pull downward)

---

## Getting Started

### Prerequisites

- GCC / Clang (C++17), CMake 3.10+, Python 3.8+

### Build and Test

```bash
mkdir build && cd build
cmake ..
cmake --build .
./unit_tests
```

### Run the Ground Control Station

```bash
cd scripts
pip install -r requirements.txt
streamlit run app.py
```

### Run the Simulation Manually

```bash
cd build
./flight_controller <Kp> <Ki> <Kd> <steps> <target1> <target2> <switch_step> <noise_sigma>
# Example:
./flight_controller 0.6 0.01 0.05 1000 50.0 100.0 500 0.5
```

`noise_sigma` is optional and defaults to 0.5 m.

---

## Features

### Mission Simulation

Two flight profiles:
- **Standard Takeoff** — ascent from 0 m to a target altitude
- **Step Response** — mid-flight altitude jump (e.g., 50 m → 100 m) to characterise rise time and agility

Metrics computed automatically: settling time (±2% band), overshoot %, RMSE.
Animated Plotly replay with DVR-style playback and CSV export.

### Noise Sensitivity Analysis

The **Noise Analysis** tab runs the simulation at five σ levels:
`[0.01, 0.1, 0.5, 1.0, 2.0] m`

For each level it collects RMSE and plots:
1. A bar chart of RMSE vs σ — shows the degradation curve
2. Overlaid altitude time-series — shows how response roughness grows with noise

This makes it straightforward to understand the trade-off between Kd (responsiveness) and
noise amplification, and to characterize how robust a set of gains is to sensor quality.

### AI Auto-Tuner

Coordinate Descent (Twiddle) optimizes Kp, Ki, Kd automatically.
- **Accuracy mode:** minimizes RMSE
- **Balanced mode:** minimizes `RMSE + 0.5 · settling_time`

Convergence criterion: `sum(dp) < 0.005`, maximum 30 iterations.
The optimizer always runs at σ = 0.5 m so gains are tuned for realistic noise conditions.

---

## Project Structure

```
├── include/
│   ├── IController.hpp     # Abstract controller interface
│   ├── ISensor.hpp         # Abstract sensor interface
│   ├── PID.hpp
│   ├── PhysicsEngine.hpp
│   └── MockSensor.hpp
├── src/
│   ├── main.cpp            # Simulation entry point (8 CLI args)
│   ├── core/PID.cpp
│   └── simulation/
│       ├── PhysicsEngine.cpp
│       └── MockSensor.cpp
├── tests/
│   ├── test_pid.cpp        # 4 PID tests
│   ├── test_physics.cpp    # 4 physics tests (use setState for clean setup)
│   └── test_sensor.cpp     # 5 MockSensor tests (noise distribution, setValue)
├── scripts/
│   └── app.py              # Streamlit GCS (Mission Simulation + Noise Analysis tabs)
├── .github/workflows/
│   └── cpp-build.yml       # CI: build+test, clang-tidy lint, ASan/UBSan
└── CMakeLists.txt          # C++17, pinned GoogleTest v1.15.2, clang-tidy, Debug sanitizers
```

---

## CI/CD

Three jobs run on every push and pull request:

| Job | What it checks |
|---|---|
| `build-and-test` | CMake Release build + all 13 unit tests |
| `lint` | clang-tidy (modernize, readability checks) during compilation |
| `sanitizer-tests` | Debug build with AddressSanitizer + UBSan; runs tests under `ASAN_OPTIONS=detect_leaks=1` |

---

## License

MIT License. Free to use for educational and portfolio purposes.
