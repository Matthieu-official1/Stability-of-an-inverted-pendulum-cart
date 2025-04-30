# Inverted Pendulum on a Cart

This project simulates and controls an inverted pendulum mounted on a cart using MATLAB. The goal is to stabilize the pendulum in its upright position through feedback control, demonstrating classical control principles such as PID and state-space design.

## 📽️ Demo

| Simulation | Real-Life Test |
|------------|----------------|
| ![Simulation](images/system_diagram.png) | ![Real Test](images/results_plot.png) |

➡️ **Watch Videos:**  
- 🎥 [Simulation Demo](videos/simulation_demo.mp4)  
- 🎥 [Real-Life Test](videos/real_life_test.mp4)

## 🛠 Features

- MATLAB-based simulation and control
- Visualization of cart-pendulum motion
- Controller tuning using pole placement and LQR
- System diagrams and plots
- Real-time performance insights (if implemented on hardware)

## 📁 Project Structure
inverted-pendulum-cart/ ├── matlab_code/ # MATLAB scripts and functions ├── images/ # Diagrams and plots ├── videos/ # Simulation and hardware test videos ├── docs/ (optional) # Reports or additional documentation ├── README.md # Project overview ├── LICENSE # License file (e.g., MIT) └── .gitignore # Files to ignore in version control

## 🔧 Requirements

- MATLAB R2021a or later
- Control System Toolbox

## ▶️ Running the Code

1. Open `main_simulation.m` in MATLAB.
2. Run the script to simulate the system.
3. Modify parameters in `controller_design.m` to tune your control system.

## 📚 Documentation

If you’re looking for a deeper explanation, check the `docs/` folder for a full project write-up.

## 📜 License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
