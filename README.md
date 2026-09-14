# Dynamic Movement — Steering Behaviors

**Type:** Individual project
**Contributor:** Carter Ward
**Course:** CS 330-1 (Artificial Intelligence / Game AI) — Program 1
**Completed:** 09/23/2025

## Purpose

This is my Program 1 submission for CS 330-1 at UAH: an implementation of "dynamic movement," where steering behaviors compute an acceleration each frame rather than setting position or velocity directly. I logged every character's state over time and plotted the resulting paths to prove the simulation works — characters accelerate/decelerate under real Newton-Euler integration, not just pseudocode.

## Problem and Approach

Four characters, each governed by a different steering behavior in a 2D plane:

- **Continue (2601)** — stationary, zero acceleration; also the target everyone else steers toward/away from.
- **Flee (2602)** — accelerates directly away from its target at max acceleration.
- **Seek (2603)** — accelerates directly toward its target at max acceleration.
- **Arrive (2604)** — like Seek, but slows inside a "slow radius" and stops inside an inner "arrive radius," using a time-to-target constant for smoothed acceleration.

I split the problem into three pieces: build the character's state, decide the acceleration it wants, then apply that acceleration — the same seek/steer/act split from lecture, which made each part easy to verify independently.

## Structure and Methodologies

- `Character` class holding position, velocity, orientation, rotation, and per-behavior tuning (max speed/acceleration, arrive/slow radii).
- 2D vectors as plain `(x, y)` tuples with small helper functions (`v_add`, `v_sub`, `v_scale`, `v_len`, `v_norm`) rather than a `Vector2` class.
- Pure steering functions (`steering_seek`, `steering_flee`, `steering_arrive`, `steering_continue`) that compute `(linear_acceleration, angular_acceleration)` without mutating state.
- A Newton-Euler-1 integrator (`integrate_NE1`) applies that acceleration to update position/velocity/orientation, clamping to max speed and zeroing near-zero speeds.
- Dependencies: `WardCS330Program1.py` uses only stdlib `math`; the separate `CS 330, Python Plotter v3_1.py` (a provided utility, not written by me) needs `matplotlib` plus stdlib `csv`/`math`.

## Process

1. Build the four `Character` objects with assignment-specified starting positions/velocities/behavior codes.
2. Write each character's t=0 state to `dynamic_trajectories.txt`.
3. For 100 steps of 0.5s: compute each character's desired acceleration, then integrate all four with Newton-Euler-1, appending state each step (404 rows total: 4 characters × 101 timestamps).
4. Run `CS 330, Python Plotter v3_1.py` against the output file to render trajectories with matplotlib, saved as `TrajectoryPlot.png`.

## Outcome

`TrajectoryPlot.png` matches the expected physics: Flee peels straight away from the origin and keeps going; Seek overshoots and spirals around the origin (expected for pure Seek with no braking term, and confirms the integration/vector math is correct); Arrive curves in and visibly slows as it enters the slow radius, proving the slow/arrive-radius logic works; Continue stays pinned at the origin throughout. The exercise reinforced separating "decide acceleration" from "apply acceleration" for easier debugging, and gave practice turning raw CSV simulation data into a visually verifiable result — a workflow I expect to reuse in later physics/AI-behavior assignments.

## How to run

```
python WardCS330Program1.py
python "CS 330, Python Plotter v3_1.py"
```

Requires `matplotlib` (`pip install matplotlib`) for the plotter; run both from the same folder.
