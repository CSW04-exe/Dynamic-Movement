# Dynamic Movement — 2D Steering Behavior Simulation

A small Python simulation of autonomous character movement using classic AI **steering behaviors** — Seek, Flee, Arrive, and Continue — driven by a Newton–Euler-1 physics integrator, with a companion Matplotlib script for visualizing the resulting trajectories.

## 1. Purpose

This project exists to demonstrate, from first principles, how simple local rules for acceleration can produce believable autonomous motion for game characters or agents — the same family of techniques (Reynolds-style steering behaviors) used in games and robotics to move non-player characters without scripting every frame of animation by hand.

It was built as **Program 1 for CS 330 (Computational Graphics and Visualization)**, an assignment focused on kinematic and dynamic movement algorithms. The point of the exercise is to implement the movement math directly — vector arithmetic, force/acceleration-based steering, and numerical integration — rather than relying on a game engine's built-in physics or navigation system, so the underlying mechanics are fully understood rather than treated as a black box.

## 2. Problem and Approach

The assignment (self-contained, course-assigned) was to simulate several characters moving in a 2D plane, each governed by a different steering behavior, and to record their motion over time so it could be verified and visualized:

- **Continue** — a character that simply keeps doing whatever it was already doing (a no-op behavior, useful as a baseline/control).
- **Flee** — accelerate directly away from a target at maximum linear acceleration.
- **Seek** — accelerate directly toward a target at maximum linear acceleration.
- **Arrive** — like Seek, but with a deceleration profile: full speed while far from the target, a linearly scaled slow-down inside a "slow radius," and a full stop inside a "stop radius," using a time constant to smooth the approach to the desired velocity.

The approach taken was to model the problem as a small discrete-event simulation:

1. Represent each character as an object carrying position, velocity, orientation, rotation, and behavior-specific tuning parameters (max speed, max acceleration, arrival radii, arrival time).
2. On every fixed time step, compute a steering output (a desired linear acceleration and angular acceleration) for each character based on its assigned behavior and, where relevant, a target character.
3. Feed that acceleration into a **Newton–Euler first-order (NE1) integrator** to update velocity and position, clamping to maximum speed and zeroing out velocities below a small threshold to prevent jitter.
4. Log every character's full state at every step to a plain-text, CSV-formatted file so the results can be inspected or plotted after the fact rather than only observed live.

Four characters were configured with different starting positions, velocities, and behaviors (one of each type) so the four behaviors could be compared side by side in the same run.

## 3. Structure and Methodologies

**Language & runtime:** pure Python 3, no external dependencies required to run the simulation itself (only the standard library `math` module is used in `WardCS330Program1.py`).

**Key data structures:**
- A `Character` class holding per-agent state: `pos`, `vel`, `ori` (orientation), `rot` (rotation), last-applied `lin_acc`/`ang_acc`, motion limits (`max_speed`, `max_linear`), and Arrive-specific parameters (`arrive_radius`, `arrive_slow`, `arrive_time`).
- 2D vectors represented simply as Python tuples `(x, y)`, manipulated through small free functions (`v_add`, `v_sub`, `v_scale`, `v_len`, `v_norm`) rather than a full vector/matrix library — a deliberate minimalism that keeps the linear algebra visible.
- A flat list of `Character` instances (`characters`) as the simulation's world state, iterated once per time step to compute steering and once more to integrate motion.
- Integer behavior codes (`CONTINUE = 1`, `SEEK = 6`, `FLEE = 7`, `ARRIVE = 8`) matching the numeric behavior IDs used in the course's data logging convention, so output rows are self-describing.

**Numerical method:** Newton–Euler-1 (semi-implicit/explicit first-order) integration — position is advanced using the *current* velocity, then velocity is advanced using the computed acceleration, each scaled by a fixed time step (`TIME_STEP_SECONDS = 0.5`) over a fixed simulated duration (`FINAL_TIME_SECONDS = 50.0`).

**Output format:** a hand-rolled CSV writer (`dynamic_trajectories.txt`) with one row per character per time step — time, ID, position (x, y), velocity (x, y), linear acceleration (x, y), orientation, behavior code, and a collision flag placeholder — 404 rows in total for this run (4 characters × 101 sampled time steps, including t = 0).

**Visualization dependency:** the companion script (`CS 330, Python Plotter v3_1.py`) uses `matplotlib` and Python's built-in `csv` module to read `dynamic_trajectories.txt` back in, group rows by character ID into a small `Mover` class, and plot each character's position trail (red), velocity vectors (green), and acceleration vectors (blue) on a shared 2D axis, producing the annotated figure saved as `TrajectoryPlot.png`.

## 4. Process

Based on the code's structure and comments, the build proceeded roughly as follows:

1. **Vector math first.** The lowest-level pieces — `v_add`, `v_sub`, `v_scale`, `v_len`, `v_norm` — were written first as small, single-purpose functions, since every steering behavior and the integrator depend on them.
2. **Character representation.** A `Character` class was defined to hold all per-agent state in one place (kinematic state, limits, and target/arrival tuning), rather than passing a long list of loose variables around.
3. **Steering behaviors implemented one at a time**, from simplest to most complex: `steering_continue` (return the previous acceleration unchanged), then `steering_seek` and `steering_flee` (mirror images of each other — accelerate toward vs. away from a target at full strength), and finally the more involved `steering_arrive`, which required adding the slow-radius/stop-radius distance checks and a time-constant-based velocity-matching calculation, plus a clamp back down to `max_linear` so Arrive never exceeds the character's acceleration budget.
4. **Integration and stability fixes.** The NE1 integrator was written to update orientation, position, and velocity each step, then two safeguards were added directly in the integrator: a small-velocity cutoff (`TINY_SPEED_THRESHOLD`) to stop characters from jittering near zero speed, and a max-speed clamp so behaviors like Flee don't accelerate a character's speed without bound.
5. **Scenario assembly.** Four `Character` instances were hand-configured with distinct starting positions, velocities, orientations, and behaviors (including one Arrive character with explicit radius/slow/time parameters) so all four behaviors would be visibly exercised in a single run.
6. **Logging and main loop.** A `write_record` function was added to serialize one character's state to a CSV line, and `main()` was written to write the initial state, then step the simulation `FINAL_TIME_SECONDS / TIME_STEP_SECONDS` times, each step computing steering for every character *before* integrating any of them (so all characters react to the same instant in time) and logging the post-step state.
7. **Verification via plotting.** Rather than trust the numbers in the log file blindly, a separate plotting script was built (adapted from an earlier "v2" version, per its header comment) to parse the CSV output and render each character's trajectory with overlaid velocity/acceleration vectors — turning a wall of numbers into a picture (`TrajectoryPlot.png`) that makes it immediately obvious whether Seek is orbiting, Flee is running away in a straight line, and Arrive is actually slowing down and stopping.
8. **Iteration on tuning constants.** Small named constants (`TINY_SPEED_THRESHOLD`, `TIME_STEP_SECONDS`, `FINAL_TIME_SECONDS`) sit at the top of the file rather than being hard-coded inline, suggesting these were tuned by re-running the simulation and re-plotting until the behaviors looked correct (no runaway jitter, Arrive settling within the run window, etc.).

## 5. Outcome

The simulation runs correctly end-to-end and produces a genuinely measurable, verifiable result in `dynamic_trajectories.txt`:

- **Arrive works as designed.** Character 2604 starts at `(50, 75)` moving at `(-9, 4)` and, under the Arrive behavior with a stop radius of 4 and slow radius of 32, decelerates smoothly and comes to a **complete stop** (`velocity = (0.000000, 0.000000)`) at position `(-0.077172, -1.758441)` by simulated time **t = 19.0s** — and stays stopped for the remaining 31 seconds of the run. That's the behavior's core promise (approach and settle, rather than overshoot or oscillate) validated directly in the output data.
- **Flee works as designed.** Character 2602 accelerates away from its target and settles into sustained near-maximum speed (velocity magnitude converges to roughly 8 units/sec, its configured `max_speed`), traveling from `(-30, -50)` out to roughly `(-335, -189)` by t = 50s — steady, unbounded retreat, as expected of a behavior with no deceleration term.
- **Seek behaves distinctly from Arrive**, as it should: with no deceleration logic, Character 2603 repeatedly closes in on its (stationary) target near the origin and overshoots, producing a looping/orbiting path rather than settling — a clear, log-verifiable illustration of *why* Arrive exists as a separate, more sophisticated behavior.
- **Continue is a correct no-op**: Character 2601, given no initial velocity, remains exactly at `(0, 0)` for all 101 logged samples, confirming the "do nothing new" behavior doesn't introduce drift.

Working through this project reinforced several concrete skills: implementing vector math and physics integration from scratch instead of leaning on a library; structuring per-agent simulation state in a way that keeps steering logic, physics, and I/O cleanly separated; and the practice of validating a simulation not just by "it ran without crashing" but by checking specific numeric invariants (does Arrive actually reach zero velocity near the target? does Flee's speed converge to the configured maximum?) and by building a visualization tool specifically to make those invariants inspectable at a glance.

More broadly, it demonstrates a problem-solving approach of decomposing a fairly open-ended "make characters move realistically" prompt into small, independently testable pieces — vector helpers, one behavior at a time, an integrator with explicit stability guards, then a separate verification/plotting pass — rather than trying to write the whole simulation as one monolithic block. The biggest takeaway was how much of "believable" AI movement comes down to just a few extra lines of deceleration math (the difference between Seek and Arrive) and careful numerical clamping (the tiny-speed cutoff and max-speed clamp) to keep a physically simulated system from jittering or diverging.
