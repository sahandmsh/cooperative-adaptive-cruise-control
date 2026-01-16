# Cooperative Adaptive Cruise Control (CACC) with MPC

A simulation of a Cooperative Adaptive Cruise Control (CACC) system using Model Predictive Control (MPC) to maintain safe spacing and velocity coordination in a vehicle platoon.

## Overview

This project implements a CACC system where multiple vehicles in a platoon coordinate their movements to maintain optimal spacing while following a lead vehicle. The control strategy uses MPC to predict future states and optimize control inputs while respecting physical constraints.

## Features

- **Model Predictive Control (MPC)**: Optimal control strategy with constraints
- **Configurable Vehicle Platoon**: Adjustable number of vehicles and parameters
- **Safety Constraints**: Enforced acceleration, velocity, and spacing limits
- **Real-time Simulation**: Step-by-step discrete-time dynamics
- **Comprehensive Visualization**: Plots for distances, velocities, and accelerations

## System Architecture

### Vehicle Dynamics

The system models two types of vehicles:

1. **Leader Vehicle**: Tracks a reference velocity
   - State: `[position, velocity, acceleration]`
   - Objective: Maintain target velocity while minimizing acceleration changes

2. **Follower Vehicles**: Maintain spacing from preceding vehicle
   - State: `[spacing_error, relative_velocity, acceleration]`
   - Objective: Minimize spacing error and relative velocity

### State-Space Representation

**Follower Vehicle Dynamics:**
```
x[k+1] = A·x[k] + B·u[k] + D·a_pred[k]
```
where:
- `x = [spacing_error, relative_velocity, acceleration]ᵀ`
- `u` = control input
- `a_pred` = acceleration of preceding vehicle

**Leader Vehicle Dynamics:**
```
x[k+1] = A·x[k] + B·u[k]
```
where:
- `x = [position, velocity, acceleration]ᵀ`

## Configuration Parameters

The `SimulationConfig` class provides comprehensive control over the simulation:

### Simulation Parameters
- `num_vehicles`: Number of vehicles in platoon (default: 5)
- `total_steps`: Simulation duration in steps (default: 150)
- `sampling_time`: Time step duration in seconds (default: 0.1s)
- `prediction_horizon`: MPC lookahead steps (default: 5)

### Vehicle Parameters
- `time_headway`: Desired time gap between vehicles (default: 0.7s)
- `vehicle_length`: Physical length of each vehicle (default: 5.0m)
- `driveline_time_constant`: Response time constant (default: 0.1s)
- `min_spacing`: Minimum bumper-to-bumper distance (default: 2.0m)

### Control Limits
- `min_acceleration`: -4.0 m/s²
- `max_acceleration`: 3.0 m/s²
- `min_control_input`: -4.0 m/s²
- `max_control_input`: 3.0 m/s²
- `max_velocity`: 35.0 m/s (~80 mph)

### Reference Tracking
- `leader_reference_velocity`: Target speed for leader (default: 30.0 m/s)

## Installation

### Requirements

```bash
pip install numpy matplotlib cvxpy
```

### Dependencies
- `numpy`: Numerical computations and array operations
- `matplotlib`: Visualization and plotting
- `cvxpy`: Convex optimization for MPC

## Usage

1. Open the Jupyter notebook `CACC-MPC.ipynb`

2. Run all cells in sequence:
   - Cell 1: Import dependencies
   - Cell 2: Define configuration and helper classes
   - Cell 3: Implement MPC solver
   - Cell 4: Initialize simulation
   - Cell 5: Run simulation loop
   - Cell 6: Visualize results

3. Customize parameters by modifying the `SimulationConfig` class or creating a custom configuration:

```python
config = SimulationConfig(
    num_vehicles=10,
    total_steps=200,
    time_headway=0.9,
    leader_reference_velocity=25.0
)
```

## MPC Optimization Problem

The MPC controller solves the following optimization at each time step:

**Objective:**

$$
\min_{u} \sum_{k=0}^{N-1} (x[k] - x_{ref})^T Q (x[k] - x_{ref}) + (x[N] - x_{ref})^T Q (x[N] - x_{ref})
$$

**Subject to:**
- State dynamics: `x[k+1] = A·x[k] + B·u[k]`
- Initial condition: `x[0] = x_current`
- Acceleration limits: `a_min ≤ a[k] ≤ a_max`
- Control limits: `u_min ≤ u[k] ≤ u_max`
- Velocity limits: `0 ≤ v[k] ≤ v_max`

where:
- `N`: prediction horizon
- `Q`: state cost matrix (different for leader/follower)
- `x_ref`: reference state to track

## Visualization

The simulation produces three plots:

1. **Inter-vehicle Distances**: Shows spacing between consecutive vehicles over time
2. **Vehicle Velocities**: Displays velocity profiles for all vehicles
3. **Vehicle Accelerations**: Shows acceleration commands for each vehicle

## Key Algorithms

### MPC Solver (`solve_mpc`)

A unified function that handles both leader and follower optimization:
- Constructs decision variables for states and controls
- Formulates constraints (dynamics, limits)
- Sets up quadratic cost function
- Solves using CVXPY
- Returns optimal trajectory

### Simulation Loop

For each time step:
1. Solve MPC for leader vehicle (velocity tracking)
2. Solve MPC for each follower (spacing control)
3. Apply first control input
4. Update vehicle states
5. Compute spacing errors

## Technical Details

### Time Discretization
The continuous-time dynamics are discretized using forward Euler method with sampling time `Δt = 0.1s`.

### Cost Matrices

**Leader Vehicle:**
- Velocity tracking weight: 5.0
- Acceleration weight: 1.0

**Follower Vehicles:**
- Spacing error weight: 5.0
- Relative velocity weight: 3.0
- Acceleration weight: 1.0

### Solver

Uses CVXPY with automatic solver selection.

## Limitations

- Assumes perfect state information (no sensor noise)
- No communication delays between vehicles
- Simplified vehicle dynamics (no steering/lateral control)
- Deterministic environment (no disturbances)

## License

This project is open source and available under the MIT License.

## Author

Sahand Mosharafian


## Acknowledgments

This code was refactored and this README was generated with assistance from GitHub Copilot.
