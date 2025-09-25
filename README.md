# PID Controller for DC Motor Speed Control (MATLAB/Simulink)

This repository demonstrates the design and analysis of a **PID Controller** applied to a **DC Motor Model** using MATLAB/Simulink.  
It investigates the effect of proportional (Kp), integral (Ki), and derivative (Kd) terms on system performance, including overshoot, rise time, settling time, and steady-state error.

The project was originally prepared as a coursework assignment and has been adapted here into a GitHub research-style project to showcase control engineering and simulation skills.

---

## Repository Structure

```
pid-motor-control/
├── README.md
├── LICENSE
├── .gitignore
├── report/
│   └── PID_Controller_Report.pdf   # Anonymized version of assignment
├── models/
│   └── dc_motor_simulink.png       # Screenshot of Simulink model
├── figures/
│   ├── step_response.png           # Example response curve
│   ├── pid_tuning_effects.png      # Performance comparison
└── scripts/
    └── pid_motor.m                 # MATLAB script for motor + PID simulation
```

---

## Project Overview

- **System**: DC motor mathematical model (transfer function based)  
- **Controller**: Classical PID control with tunable Kp, Ki, Kd  
- **Tool**: MATLAB/Simulink  

This project explores how varying PID parameters impacts system dynamics, including overshoot, oscillations, and settling behavior.

---

## Key Features

1. **DC Motor Modeling**
   - Transfer function representation  
   - Mathematical derivation of system dynamics  

2. **PID Control Design**
   - Tuning Kp, Ki, Kd to achieve desired response  
   - Investigation of parameter sensitivity  

3. **Simulation & Results**
   - Step response of DC motor with PID  
   - Effect of Kp increase: faster response but overshoot  
   - Effect of Ki increase: reduces steady-state error but causes oscillations  
   - Effect of Kd: improves stability and reduces overshoot  

---

## Example Figures in report

- Simulink model screenshot  
- Step response curve (PID tuned vs untuned)  
- Performance comparison for different Kp, Ki values  

---

## Example MATLAB Snippet

```matlab
% DC Motor Transfer Function
J = 0.01;   % inertia
b = 0.1;    % damping
K = 0.01;   % motor constant
R = 1;      % resistance
L = 0.5;    % inductance

num = K;
den = [L*J (R*J+L*b) (R*b+K^2)];
motor_tf = tf(num, den);

% PID Controller
Kp = 100; Ki = 200; Kd = 10;
C = pid(Kp, Ki, Kd);

% Closed-loop system
sys_cl = feedback(C*motor_tf, 1);
step(sys_cl);
title('DC Motor with PID Control');
```

---

## How to Use

1. Open the `report/PID_Controller_Report.pdf` for full analysis.  
2. View `models/dc_motor_simulink.png` to understand the block diagram structure.  
3. Run MATLAB/Simulink with the provided transfer function and PID controller.  
4. Tune Kp, Ki, Kd to observe system performance differences.  

---

## Notes

- Original report was anonymized (student ID and declaration removed).  
- This repository is intended as an **educational portfolio project**.  
- Results demonstrate practical control system design and analysis.  

---

## Author

**Jinyan Yang** — MSc in Renewable Energy Systems
