# ----------------------------------------------------------------
# Author : Carter Ward
# Class  : CS 330-1
# Date   : 09/23/25
#
# Simulates 4 characters moving in 2D space with Seek, Flee, Arrive,
# and Continue behaviors. Uses Newton–Euler-1 to update position and
# velocity, and writes the results to a text file.
# ----------------------------------------------------------------

import math

# -------------------------
# Behavior codes
# -------------------------
CONTINUE = 1
SEEK     = 6
FLEE     = 7
ARRIVE   = 8

# -------------------------
# Simulation settings
# -------------------------
TINY_SPEED_THRESHOLD = 0.02  # ignore tiny movement (avoid jitter)
TIME_STEP_SECONDS    = 0.50  # step size
FINAL_TIME_SECONDS   = 50.0  # stop after 50s

# -------------------------
# Simple vector helpers
# -------------------------
def v_add(a, b): return (a[0] + b[0], a[1] + b[1])       # add two vectors
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1])       # subtract b from a
def v_scale(v, s): return (v[0] * s, v[1] * s)           # scale vector by s
def v_len(v): return math.hypot(v[0], v[1])              # vector length
def v_norm(v):                                           # normalize vector
    L = v_len(v)
    return (0.0, 0.0) if L == 0 else (v[0] / L, v[1] / L)

# -------------------------
# Character definition
# -------------------------
class Character:
    def __init__(self, character_id, steering_code,
                 position=(0.0, 0.0), velocity=(0.0, 0.0),
                 orientation=0.0, rotation=0.0,
                 max_speed=0.0, max_linear=0.0,
                 target_index=0, arrive_radius=0.0,
                 arrive_slow=0.0, arrive_time=0.0):
        # basic state
        self.id = character_id
        self.steer = steering_code
        self.pos = position
        self.vel = velocity
       
        # motion/rotation
        self.ori = orientation
        self.rot = rotation
        
        # acceleration from last step
        self.lin_acc = (0.0, 0.0)
        self.ang_acc = 0.0
       
        # limits
        self.max_speed = max_speed
        self.max_linear = max_linear
        
        # targeting + arrive params
        self.target_index = target_index
        self.arrive_radius = arrive_radius
        self.arrive_slow = arrive_slow
        self.arrive_time = arrive_time
        
        # collision flag (not used here)
        self.collided = False

# -------------------------
# Steering behaviors
# -------------------------
def steering_continue(ch):
    # keep doing what you were doing last frame
    return (ch.lin_acc, ch.ang_acc)

def steering_seek(ch, target):
    # accelerate toward target
    d = v_sub(target.pos, ch.pos)
    return (v_scale(v_norm(d), ch.max_linear), 0.0)

def steering_flee(ch, target):
    # accelerate away from target
    d = v_sub(ch.pos, target.pos)
    return (v_scale(v_norm(d), ch.max_linear), 0.0)

def steering_arrive(ch, target):
    # like seek, but slow down when close
    d = v_sub(target.pos, ch.pos)
    dist = v_len(d)
    if dist < ch.arrive_radius:         # inside stop radius
        speed = 0.0
    elif dist > ch.arrive_slow:         # far away → full speed
        speed = ch.max_speed
    else:                               # in slow-down zone
        speed = ch.max_speed * (dist / ch.arrive_slow)
    desired_vel = v_scale(v_norm(d), speed)
    tau = ch.arrive_time if ch.arrive_time > 0 else 1e-9
    lin = v_scale(v_sub(desired_vel, ch.vel), 1.0 / tau)
    if v_len(lin) > ch.max_linear:      # clamp if too strong
        lin = v_scale(v_norm(lin), ch.max_linear)
    return (lin, 0.0)

# -------------------------
# Integrator (Newton–Euler-1)
# -------------------------
def integrate_NE1(ch, lin_acc, ang_acc, dt):
    # update position from velocity
    ch.pos = v_add(ch.pos, v_scale(ch.vel, dt))
    
    # wrap orientation to 0–2π
    ch.ori = (ch.ori + ch.rot * dt) % (2.0 * math.pi)
   
    # update velocity from acceleration
    ch.vel = v_add(ch.vel, v_scale(lin_acc, dt))
    
    # update rotation from angular accel
    ch.rot = ch.rot + ang_acc * dt
    
    # store accelerations for output
    ch.lin_acc = lin_acc
    ch.ang_acc = ang_acc
    
    # kill very small velocity (prevents jitter)
    if v_len(ch.vel) < TINY_SPEED_THRESHOLD:
        ch.vel = (0.0, 0.0)
   
    # enforce max speed limit
    if ch.max_speed > 0 and v_len(ch.vel) > ch.max_speed:
        ch.vel = v_scale(v_norm(ch.vel), ch.max_speed)

# -------------------------
# Scenario setup (4 characters)
# -------------------------
characters = [
    Character(2601, CONTINUE, (0, 0)),                                # stationary continue
    Character(2602, FLEE, (-30, -50), (2, 7), math.pi/4, 0, 8, 1.5),  # fleeing
    Character(2603, SEEK, (-50, 40), (0, 8), 3*math.pi/2, 0, 8, 2.0), # seeking
    Character(2604, ARRIVE, (50, 75), (-9, 4), math.pi, 0, 10, 2.0,
              0, 4, 32, 1)                                            # arriving
]

# -------------------------
# Write CSV line for one character
# -------------------------
def write_record(fh, t, ch):
    row = [
        f"{t:.2f}", str(ch.id),
        f"{ch.pos[0]:.6f}", f"{ch.pos[1]:.6f}",
        f"{ch.vel[0]:.6f}", f"{ch.vel[1]:.6f}",
        f"{ch.lin_acc[0]:.6f}", f"{ch.lin_acc[1]:.6f}",
        f"{ch.ori:.6f}", str(ch.steer), "FALSE"
    ]
    fh.write(",".join(row) + "\n")

# -------------------------
# Main simulation
# -------------------------
def main():
    with open("dynamic_trajectories.txt", "w", encoding="utf-8") as fh:
        t = 0.0
        # write initial state before stepping
        for ch in characters:
            write_record(fh, t, ch)

        steps = int(FINAL_TIME_SECONDS / TIME_STEP_SECONDS)
        for _ in range(steps):
            t += TIME_STEP_SECONDS
            steering_results = []
            
            # figure out what each character wants to do
            for ch in characters:
                if ch.steer == CONTINUE:
                    s = steering_continue(ch)
                elif ch.steer == SEEK:
                    s = steering_seek(ch, characters[ch.target_index])
                elif ch.steer == FLEE:
                    s = steering_flee(ch, characters[ch.target_index])
                elif ch.steer == ARRIVE:
                    s = steering_arrive(ch, characters[ch.target_index])
                else:
                    s = ((0.0, 0.0), 0.0)  # default no accel
                steering_results.append(s)

            # apply physics update
            for ch, (lin, ang) in zip(characters, steering_results):
                integrate_NE1(ch, lin, ang, TIME_STEP_SECONDS)

            # write state after step
            for ch in characters:
                write_record(fh, t, ch)

    print("Wrote trajectories to dynamic_trajectories.txt")

if __name__ == "__main__":
    main()
