
# DC Motor Control Simulator GUI

## Overview

`motor_control_gui` is a MATLAB graphical user interface (GUI) for simulating a DC motor control system with PID controllers. The GUI allows users to:

- Input motor and PID controller parameters.
- Run motor control simulations with different controller types (open-loop, P, PI, PID).
- Load experimental motor speed data from CSV files.
- Visualize step responses and PID disturbance rejection.
- Save simulation results to CSV files.
- Reset parameters to default values.

---

## Key Features

- Interactive inputs for motor electrical and mechanical parameters.
- Adjustable PID controller gains (Proportional, Integral, Derivative).
- Plots showing motor speed responses over time.
- Comparison of simulated results with experimental data.
- User-friendly buttons to control simulation flow and data management.

---

## Description of Parameters

| Parameter           | Symbol | Unit          | Description                                     | Default Value |
|---------------------|--------|---------------|------------------------------------------------|---------------|
| Resistance          | R      | Ohms (Ω)      | Motor winding resistance                        | 1             |
| Inductance          | L      | Henry (H)     | Motor winding inductance                        | 0.5           |
| Back EMF constant   | Ke     | V·s/rad       | Voltage constant related to motor speed         | 0.01          |
| Torque constant     | Kt     | Nm/A          | Motor torque per ampere                          | 0.01          |
| Inertia             | J      | kg·m²         | Motor rotor inertia                             | 0.01          |
| Damping coefficient | b      | Nms           | Mechanical damping                             | 0.1           |
| PID Proportional gain  | Kp     | -             | Gain for proportional control component          | 100           |
| PID Integral gain      | Ki     | -             | Gain for integral control component               | 200           |
| PID Derivative gain    | Kd     | -             | Gain for derivative control component             | 1             |
| Simulation time vector | t      | seconds       | Time points for simulation steps                  | 0 to 2 s in 0.01 increments |

---

## GUI Components

### Input Fields

- Text boxes for entering motor parameters: R, L, Ke, Kt, J, b.
- Text boxes for PID gains: Kp, Ki, Kd.

### Buttons

| Button                  | Description                                                |
|-------------------------|------------------------------------------------------------|
| **Run Simulation**      | Runs the motor control simulation with current parameters. |
| **Load Experimental Data** | Load time-speed data from a CSV file for comparison.      |
| **Reset to Defaults**   | Resets all inputs and clears data to initial default values.|
| **Save Simulation Data**| Save current simulation results to a CSV file.             |

### Plot Axes

- **Step Response Plot:** Shows speed response curves for open-loop, P, PI, and PID controllers.
- **Disturbance Rejection Plot:** Shows motor speed response under PID controller with disturbance input.

### Status Bar

- Displays feedback messages to the user (errors, success notices).

---

## Functions Explained

### `readParams`

Reads and validates user inputs from the GUI fields:

- Converts input strings to numeric values.
- Checks for valid numeric input.
- Ensures physical parameters (R, L, J) are positive.
- Checks PID gains (Kp, Ki, Kd) are non-negative.
- Returns parameters struct and error message if validation fails.

---

### `runSimulation`

- Reads current parameters via `readParams`.
- Builds motor transfer function based on parameters.
- Calculates step responses for open-loop, P, PI, and PID controllers.
- Plots these responses.
- Simulates and plots disturbance rejection under PID control.
- Updates status bar with success or error messages.

---

### `loadData`

- Opens file dialog to select CSV file.
- Loads time and speed columns from CSV.
- Displays error if file format is invalid.
- Adds experimental data to plots for comparison.
- Updates status bar with load status.

---

### `resetDefaults`

- Resets all input fields to default parameters.
- Clears stored experimental and simulation data.
- Clears plots and legends.
- Updates status bar.

---

### `saveData`

- Saves latest simulation results (time, open-loop, P, PI, PID speed) to CSV.
- Opens file dialog for user to specify save location.
- Displays error if no simulation data available or on save failure.
- Updates status bar.

---

## Usage Notes

- Make sure all input fields contain numeric values before running simulation.
- Load experimental data CSV files with at least two columns: time and speed.
- Use the **Reset to Defaults** button to clear any changes and start fresh.
- Save simulation data to keep a record or for further analysis.

