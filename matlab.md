# MATLAB Simulation of an Inverted Pendulum

This project demonstrates a **MATLAB Simulink-based simulation of an inverted pendulum system** — a classic control systems problem. The system comprises a **cart-pendulum setup** where a force is applied to the cart, and the behavior of the pendulum is observed through simulation.

## System Overview

- **Input:**

  - A single input: `Force` applied to the cart.

- **Outputs:**

  - `x`: The **horizontal position** of the cart.
  - `θ`: The **angle** of the pendulum from the vertical.

## ⚙️ Simulation Details

The system internally uses **four calculated parameters/functions** to derive the pendulum’s motion:

1. `ẍ` – Acceleration of the cart.
2. `θ̈` – Angular acceleration of the pendulum.
3. `N` – The **net force acting on the pendulum in the x-direction**.
   - Formula: `N = m(ẍ − lθ̇²sinθ + lθ̈cosθ)`
4. `P` – The **net force acting on the pendulum in the y-direction**.
   - Formula: `P = m(lθ̇²cosθ + lθ̈sinθ + g)`

All of these are passed through integrators to compute `x` and `θ` over time.

## Kalman Filter Enhancement

To improve the accuracy and filter out noise, a **Kalman Filter** is incorporated:

- Two white noise blocks simulate sensor noise.
- Adders introduce this noise into the system.
- The Kalman block estimates the true state variables (`x̂`), refining the output.

## Visualization

A **Scope** block at the end shows the simulation output. The graph shows how `x` and `θ` evolve over a 10-second simulation window, along with the impact of noise and filtering.

## ⚖️ Physical Parameters Used:

| Symbol | Description                           | Value        |
|--------|---------------------------------------|--------------|
| `M`    | Mass of the cart                      | 0.5 kg       |
| `m`    | Mass of the pendulum                  | 0.2 kg       |
| `b`    | Friction coefficient of the cart      | 0.1 N/m/sec  |
| `l`    | Distance to pendulum's center of mass | 0.3 m        |
| `I`    | Inertia of the pendulum               | 0.006 kg·m²  |

## Attached Files

- `structure.jpeg`: Simulink model overview.
- `simulation.jpeg`: Scope output.
- 'inverted_pendulum.slx':Matlab file 

---

