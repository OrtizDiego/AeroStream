# AeroStream: Autonomous Flight Control & Telemetry System
![C++ Build Check](https://github.com/OrtizDiego/AeroStream/actions/workflows/cpp-build.yml/badge.svg)

A high-performance flight control simulation implemented in **C++** with a **Python** telemetry analysis suite. This project demonstrates modular systems architecture, PID control theory, and hardware abstraction.

## 🚀 Key Features
* **PID Control Core:** Real-time altitude control with tuned proportional, integral, and derivative gains.
* **Hardware Abstraction Layer (HAL):** Uses C++ Interfaces (Abstract Classes) to allow seamless swapping between Mock Sensors and real hardware.
* **Sensor Noise Simulation:** Implements stochastic noise modeling to test controller robustness.
* **Telemetry Pipeline:** Python-based data visualization for post-flight analysis.

## 🛠 Tech Stack
* **Language:** C++17, Python 3.10
* **Build System:** CMake
* **Libraries:** Pandas, Matplotlib (Python), STL (C++)

## 📊 System Architecture


The system follows a classic closed-loop feedback architecture:
1. **MockSensor** generates a noisy altitude reading.
2. **PID Class** calculates the required motor power to reach the `target_altitude`.
3. **Main Loop** updates the physics simulation and logs data to `telemetry.csv`.
4. **Python Script** parses the CSV to visualize the settling time and overshoot.

## 📈 Results
Current performance shows a settling time of ~9 seconds with an overshoot of 1%. 
<img width="1000" height="600" alt="result" src="https://github.com/user-attachments/assets/bad22812-d47d-4343-9cd4-4f5c470ebca9" />

## 📦 How to Run
1. **Build C++:**
   ```bash
   mkdir build && cd build
   cmake ..
   cmake --build .
   ./flight_controller
