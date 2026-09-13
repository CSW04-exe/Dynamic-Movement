# Dynamic-Movement

## Purpose

This is my Program 1 submission for CS 330-1 at UAH. The assignment was to implement "dynamic movement" — steering behaviors that compute an acceleration for an agent each frame rather than setting its position or velocity directly — and to prove the simulation works by logging every character's state over time and visualizing the resulting paths. I built it to show that I understand the Seek/Flee/Arrive/Continue behaviors from a physics standpoint, not just as pseudocode: the characters actually move under Newton–Euler integration, accelerate and decelerate correctly, and their trajectories look the way the underlying math predicts.

## Problem and Approach

The assignment called for four characters, each governed by a different steering behavior, moving in a 2D plane:

- **Continue (2601)** — a stationary character that just keeps doing whatever it was already doing (zero acceleration, so it sits at the origin the whole run). It also doubles as the target every other character steers toward or away from.
- **Flee (2602)** — accelerates directly away from its target at max acceleration.
- **Seek (2603)** — accelerates directly toward its target at max acceleration.
- **Arrive (2604)** — like Seek, but scales its desired speed down inside a "slow radius" and stops inside an inner "arrive radius," using a time-to-target constant to compute a smoothed acceleration instead of snapping straight to the desired velocity.

My approach was to separate the problem into three independent pieces so each could be reasoned about (and debugged) on its own:

1. A small `Character` class holding position, velocity, orientation, rotation, and the per-behavior tuning parameters (max speed, max acceleration, arrive radius/slow radius/arrive time).
2. Pure steering functions (`steering_seek`, `steering_flee`, `steering_arrive`, `steering_continue`) that take a character and a target and return `(linear_acceleration, angular_acceleration)` — they never mutate state, they just compute what the character *wants* to do this frame.
3. A Newton-Euler-1 integrator (`integrate_NE1`) that actually applies that acceleration to update position, velocity, and orientation, then clamps velocity to `max_speed` and zeroes out anything below a tiny-speed threshold so characters don't jitter forever instead of coming to rest.

Keeping "decide acceleration" and "apply acceleration" as separate steps is the same seek/steer/act split from the DM lecture, and it made it much easier to verify each behavior's math independently before trusting the full simulation loop.

## Structure and Methodologies

- **State representation**: instead of a full vector class, I represent 2D vectors as plain `(x, y)` tuples and wrote five helper functions (`v_add`, `v_sub`, `v_scale`, `v_len`, `v_norm`) to do vector math on them. It's a lighter-weight choice than a `Vector2` class, but it keeps every steering function short and readable.
- **Simulation loop**: `WardCS330Program1.py` (the only file with a hard dependency — just `math` from the standard library, no third-party packages) runs a fixed-timestep loop: `TIME_STEP_SECONDS = 0.5`, `FINAL_TIME_SECONDS = 50.0`, so 100 integration steps per character. Each step first computes every character's desired acceleration from its behavior, *then* integrates all four characters — so no character's steering decision this frame depends on another character's already-updated position, matching Newton-Euler-1's frame-based update model.
- **Output format**: every character's state (at every timestep, including t=0) is appended to `dynamic_trajectories.txt` as one CSV row: `time, id, pos_x, pos_y, vel_x, vel_y, lin_acc_x, lin_acc_y, orientation, behavior_code, collided`. The `collided` column is always `FALSE` — collision detection wasn't part of this assignment, but the column was left in to match the data schema the plotter expects.
- **Plotting**: `CS 330, Python Plotter v3_1.py` is a separate script (not something I wrote from scratch — it's a provided plotting utility I ran against my output) that depends on `matplotlib` (`pip install matplotlib`) plus the standard `csv` and `math` modules. It reads `dynamic_trajectories.txt` back in, buckets rows by character ID into a `Mover` object, and for each mover draws its position trail (red), short velocity vectors scaled 2x for visibility (green), and linear-acceleration vectors (blue), labeling each trail with its behavior name.

## Process

1. Run `WardCS330Program1.py`. It builds the four `Character` objects with the starting positions/velocities/behavior codes specified for the assignment (e.g. Flee starts at (-30, -50) with velocity (2, 7); Arrive starts at (50, 75) with velocity (-9, 4) and arrive/slow radii of 4 and 32).
2. It writes the t=0 state for all four characters to `dynamic_trajectories.txt` first.
3. Then, for 100 steps of 0.5s each: it asks every character's behavior function what acceleration it wants (Seek/Flee toward or away from character 2601, Arrive toward 2601 with slowdown, Continue just repeats its last acceleration), applies Newton-Euler-1 integration to every character using that acceleration, and appends the post-step state of all four characters to the file — so the file ends up with 404 rows (4 characters × 101 timestamps).
4. Separately, I ran `CS 330, Python Plotter v3_1.py` in the same folder as the generated `dynamic_trajectories.txt`. It parses the CSV, reconstructs each character's full trail, and renders it with matplotlib, producing the figure saved here as `TrajectoryPlot.png`.

## Outcome

`TrajectoryPlot.png` matches what the physics should produce for each behavior:

- **Flee** peels straight away from the origin and keeps going — no target-seeking pull, so it never turns back.
- **Seek** doesn't calmly walk up to the origin and stop — because `steering_seek` always accelerates at full `max_linear` toward the target with no braking term, it overshoots, and the plot shows it spiraling in a loop around the origin instead of settling there. That's the expected (if slightly chaotic-looking) behavior of pure Seek without an arrive radius, and seeing it show up in the plot confirmed my integration and vector math were correct rather than pointing to a bug.
- **Arrive** curves in toward the origin and visibly slows its velocity vectors down as it enters the slow radius, which is the whole point of Arrive over Seek — this is the clearest visual proof the slow-radius/arrive-radius logic in `steering_arrive` is working.
- **Continue** stays pinned at the origin the entire run, which is correct since it starts with zero velocity and zero acceleration and nothing else changes it.

Working through this assignment tightened up my understanding of vector-based motion: separating "what acceleration do I want" from "how do I integrate that into position/velocity" made it obvious where a bug would live if the trajectories looked wrong, and watching Seek spiral instead of stopping was a good concrete reminder of why Arrive exists as its own behavior. It also gave me practice reading raw simulation data back out of a CSV log and turning it into something visually verifiable, which is a workflow I expect to reuse for future physics-based or AI-behavior assignments in this course.

## How to run

```
python WardCS330Program1.py
```

This regenerates `dynamic_trajectories.txt`. Then, with `matplotlib` installed (`pip install matplotlib`), run:

```
python "CS 330, Python Plotter v3_1.py"
```

from the same folder to view the trajectory plot (the script calls `plt.show()`; uncomment the `plt.savefig(...)` line if you want it written to a file like `TrajectoryPlot.png` instead).
