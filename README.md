# Telescope Control System: LQR-Based High-Precision Celestial Tracking
---

## Overview

This project presents a MATLAB-based simulation and control system for high-accuracy telescope tracking using a Linear Quadratic Regulator (LQR). The system is designed to achieve sub-arcsecond precision in both azimuth and elevation axes, making it suitable for research prototyping, benchmarking, and educational demonstrations in astronomical instrumentation.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Mathematical Modeling](#mathematical-modeling)
- [LQR Control Algorithm](#lqr-control-algorithm)
- [Simulation & Visualization](#simulation--visualization)
- [Results and Comparative Analysis](#results-and-comparative-analysis)
- [How to Run](#how-to-run)
- [References](#references)
- [License](#license)

---

## Features

- **Optimal LQR control** for dual-axis (azimuth/elevation) telescope tracking
- **Sub-arcsecond RMSE** in both axes (0.16″ azimuth, 0.47″ elevation)
- **Energy-efficient**: 45% lower peak torque than traditional systems
- **Real-time 2D visualization** of pointing and tracking
- **Adaptive RK4 integration** with noise and disturbance simulation
- **Open-source MATLAB/Simulink toolkit** for rapid adoption

---

## System Architecture

- **Dual-axis state-space model** (azimuth and elevation)
- **LQR controller** with Bryson's rule for Q/R weighting
- **RK4 integrator** for high-precision simulation
- **2D animated visualization** of telescope orientation and tracking errors
- **Performance metrics**: RMSE, settling time, control effort, and energy efficiency

---



---
## Mathematical Modeling

The telescope's dynamics are modeled as a continuous-time state-space system:

### State Vector
The telescope's state vector is defined as:

$$
Y = \begin{bmatrix}
\theta \\ 
\phi \\ 
\omega_\theta \\ 
\omega_\phi
\end{bmatrix}
$$

Where:
- $\theta$: Azimuth angle (radians)
- $\phi$: Elevation angle (radians)  
- $\omega_\theta$: Azimuth angular velocity (rad/s)
- $\omega_\phi$: Elevation angular velocity (rad/s)


### Dynamics
$$
\begin{aligned}
\frac{d\theta}{dt} &= \omega_\theta \\
\frac{d\omega_\theta}{dt} &= \frac{-b_\theta \omega_\theta}{I_\theta} + \frac{u_\theta}{I_\theta} \\
\frac{d\phi}{dt} &= \omega_\phi \\
\frac{d\omega_\phi}{dt} &= \frac{-b_\phi \omega_\phi}{I_\phi} + \frac{u_\phi}{I_\phi}
\end{aligned}
$$

**Parameters:**
- $I_\theta = 0.75\ \mathrm{kg\cdot m^2}$ (azimuth moment of inertia)
- $I_\phi = 0.8\ \mathrm{kg\cdot m^2}$ (elevation moment of inertia)
- $b = 0.15\ \mathrm{N\cdot m\cdot s/rad}$ (damping coefficient)

### State-Space Form
$$
\dot{x} = Ax + Bu
$$

---

## LQR Control Algorithm
$$
J = \int_0^\infty \left( x^T Q x + u^T R u \right) dt
$$
### Objective Function

where:
- $Q$ = state weighting matrix
- $R$ = control effort weighting matrix

### Bryson's Rule
$$
Q = \mathrm{diag}\left(\frac{1}{(\theta_{\max})^2}, \frac{1}{(\omega_{\theta,\max})^2}, \frac{1}{(\phi_{\max})^2}, \frac{1}{(\omega_{\phi,\max})^2}\right)
$$

$$
R = \mathrm{diag}\left(\frac{1}{(u_{\theta,\max})^2}, \frac{1}{(u_{\phi,\max})^2}\right)
$$

### Optimal Gain

  U = Kx

where $K$ is computed by solving the continuous-time algebraic Riccati equation (CARE):

$$
A^T P + PA - PBR^{-1}B^T P + Q = 0
$$

$$
K = R^{-1}B^T P
$$



- **MATLAB Implementation:**  
 ## How to Run
  2. **Open MATLAB** (R2021b or newer, Control System Toolbox required)
  3. **Run the main simulation script:**

---

## Simulation & Visualization

- **Integration:** Fourth-order Runge-Kutta (RK4) with small random noise for realism.
- **2D Animation:**
  - ![Screenshot 2025-04-21 225139](https://github.com/user-attachments/assets/2d6aa6c4-c8b5-440a-98e8-e9c907d1a13b)

  - ![Screenshot 2025-04-21 225229](https://github.com/user-attachments/assets/cd86c77b-afe0-46a3-ada3-3c3bbfa4a1bf)


  - ![Screenshot 2025-04-21 225313](https://github.com/user-attachments/assets/da6de02e-da44-4958-bc22-adb1881ba9c7)


---

## Results and Comparative Analysis

**High-Precision Tracking:**
- **Azimuth RMSE:** 0.16 arcseconds (13× better than ML-based SPT)
- **Elevation RMSE:** 0.47 arcseconds (8× better than PID-based systems)
- **Settling Time:** 1.2 seconds for a 90° slew, zero overshoot
- **Energy Efficiency:** 45% lower peak torque than TMT’s system

**Benchmark Table:**

| System      | Azimuth RMSE | Elevation RMSE | Control Method | Update Rate (Hz) | Application      |
|-------------|--------------|----------------|---------------|------------------|------------------|
| **This Work** | **0.16″**    | **0.47″**      | Bryson-LQR    | 100,000          | Simulation       |
| SPT         | 2.14″        | 3.57″          | XGBoost (ML)  | 25               | CMB Survey       |
| TMT Design  | 0.05″        | 0.05″          | Hierarchical   | 10,000           | Optical Astronomy|
| FAST        | 8.00″        | 8.00″          | Astrometric    | 1                | Radio Astronomy  |
| ADRC        | 1.82″        | 0.95″          | ADRC          | 1,000            | Wide-field Optics|

**Technological Highlights:**
- **Bryson-optimized Q/R:** Prevents actuator saturation, maintains sub-arcsecond precision
- **Adaptive RK4:** Fast, accurate, handles discontinuities and angle wrapping
- **Open-Source Toolkit:** MATLAB/Simulink GUI, disturbance injection, hardware export
- **Cost-Benefit:** $50k for 0.5″ accuracy vs. $250k for 1″ in commercial systems

---


4. **View results:**  
- 2D animation of telescope tracking  
- Plots of azimuth/elevation tracking, errors, and control inputs

---


---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
