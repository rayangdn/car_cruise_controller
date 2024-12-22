# Model Predictive Control Project - Highway Cruise Control

This project implements various Model Predictive Control (MPC) strategies for a car driving on a highway, focusing on cruise control and vehicle following scenarios.

## Project Structure

The project is divided into six main parts:

1. **System Dynamics**: Understanding and simulating the nonlinear car model
2. **Linearization**: Deriving and implementing linearized dynamics around steady states
3. **Linear MPC**: Designing controllers for longitudinal and lateral motion
4. **Offset-Free Tracking**: Implementing disturbance estimation and rejection
5. **Robust Tube MPC**: Developing adaptive cruise control with safety guarantees
6. **Nonlinear MPC**: Creating controllers for tracking and overtaking maneuvers

## Setup Requirements

Before running the project, ensure you have installed:
- YALMIP
- MPT3
- Gurobi
- CasADi

All setup files should be installed according to the course exercise setup instructions on Moodle.

## Project Files

The main components include:
- `Car.m`: Main car model class
- `MpcControl_lon.m`: Longitudinal control implementation
- `MpcControl_lat.m`: Lateral control implementation
- `LonEstimator.m`: State estimation for offset-free tracking
- `NmpcControl.m`: Nonlinear MPC implementation
- `tube_mpc_sets.m`: Robust tube MPC computations

## Key Features

- Linear and nonlinear MPC implementations
- Robust control with guaranteed safety constraints
- Offset-free tracking capabilities
- Overtaking maneuvers with collision avoidance
- Adaptive cruise control functionality

## System Constraints

The system operates under the following constraints:
- Lane boundaries: -0.5m ≤ y ≤ 3.5m
- Heading angle: |θ| ≤ 5°
- Throttle input: -1 ≤ uT ≤ 1
- Steering angle: |δ| ≤ 30°

## Running Simulations

Basic simulation can be run using:

```matlab
Ts = 1/10;  % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120/3.6);  % Compute steady state
sys = car.linearize(xs, us);  % Linearize the model
```

For specific simulation scenarios, refer to the individual parts in the project description.

## Project Dependencies

- MATLAB (recent version recommended)
- Control System Toolbox
- Optimization Toolbox
- Required third-party tools:
  - YALMIP
  - MPT3
  - Gurobi
  - CasADi
