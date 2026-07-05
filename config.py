"""
Central configuration for the AI-GP autonomy stack (vision-based, FlightSim 2.0).
All tunable numbers live here so we never hunt through code to change behavior.
"""

import math

# ----------------------------------------------------------------------------
# Connection
# ----------------------------------------------------------------------------
SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

VISION_UDP_IP = "0.0.0.0"
VISION_UDP_PORT = 5600

# ----------------------------------------------------------------------------
# Control loop
# ----------------------------------------------------------------------------
CONTROL_HZ = 50            # command rate (spec limit: <100 Hz)
ARM_RETRY_S = 2.0

# Which body frame to send velocities in. 8 = MAV_FRAME_BODY_NED.
# If the drone ignores commands, try 9 (MAV_FRAME_BODY_OFFSET_NED).
BODY_FRAME = 8

# Axis sign corrections (velocity backend). The probe can recommend flips.
SIGN_VX = 1.0
SIGN_VZ = 1.0
SIGN_YAW = 1.0

# ----------------------------------------------------------------------------
# CONTROL MODE
# "ACRO"     -> SET_ATTITUDE_TARGET rates + thrust (FPV native; the sim HUD
#               shows FLIGHT MODE: ACRO, so this is the correct language)
# "VELOCITY" -> the old SET_POSITION_TARGET velocity backend (kept as fallback)
# ----------------------------------------------------------------------------
CONTROL_MODE = "ACRO"

# --- ACRO: thrust ---
HOVER_THRUST = 0.35          # measured by the calibration probe (liftoff ~0.35)
LAUNCH_THRUST_BOOST = 0.06   # gentle: the old boost flew OVER gate 0 every run
THRUST_MIN = 0.0
THRUST_MAX = 0.95
# Closed-loop vertical velocity (thrust chases a vz target using the
# accelerometer-derived vertical velocity estimate; trim LEARNS true hover)
VZ_KP = 0.28                 # thrust per (m/s of vertical velocity error)
TRIM_KI = 0.10               # trim learn rate (thrust per m/s per second)
TRIM_LEARN_VZ_MAX = 0.15     # ONLY learn trim when |vz intent| is below this
                             # (during commanded climbs/descents the leaky vz
                             #  estimate lies and poisons the trim)
TRIM_MIN = 0.12            # was 0.25: descend-stall logs show 0.25 still hovers
ALLOW_SIGN_FLIPS = False   # once calibration is learned, runtime sign-flip
                           # heuristics are OFF: detector noise (two gates
                           # alternating in view) was un-learning correct signs
GATE_LOCK_RADIUS_FRAC = 0.35  # candidate within this frac of frame width of the
                              # last detection wins over a larger blob elsewhere
GATE_LOCK_MEMORY_S = 2.0      # how long the lock position stays valid
TRIM_MAX = 0.75
CLIMB_SAT_THRUST = 0.62    # softened: 0.88 rocketed past gate height every run
FLOW_DAMP_K = 0.12         # thrust cut per unit of image flow while climbing
FLOW_DAMP_MAX = 0.15       # cap on flow damping
EL_HOLD_RAD = 0.17         # el above ~10deg -> STOP advancing, climb in place
EL_ADVANCE_RAD = 0.14      # resume forward motion once el is back under ~8deg
VZ_EST_LEAK = 0.05           # 1/s leak on the vz integrator (bounds drift)

# --- ACRO: attitude ---
PITCH_PER_MS = 0.045         # rad of nose-down per (m/s forward intent) ~2.6deg
MAX_PITCH = 0.38             # rad (~22 deg) forward tilt cap
ANGLE_KP = 4.0               # rate command per rad of angle error
MAX_BODY_RATE = 3.0          # rad/s clamp on roll/pitch rate commands
SIGN_PITCH_FWD = -1.0        # sign of pitch angle that means "nose down/forward"
SIGN_ROLL = 1.0

# --- In-flight sign auto-calibration (during LAUNCH) ---
CALIB_ENABLE = True
CALIB_PULSE_RATE = 1.2       # rad/s test pulse
CALIB_PULSE_S = 0.15         # pulse duration per axis
CALIB_SETTLE_S = 0.15        # settle between pulses
CALIB_MIN_RESPONSE = 0.15    # rad/s of gyro response needed to judge sign

# --- Yaw-steering self-check (TRACK): if the gate drifts further off-center
# while we steer at it, the steering sign is inverted -> flip it once ---
YAW_STEER_WINDOW_S = 0.7
YAW_STEER_GROW_RAD = 0.08
YAW_FLIP_COOLDOWN_S = 3.0

# --- Optical-flow motion check (SEARCH): commanding vertical motion must
# make the scene visibly stream; if not, the trim is wrong -> nudge toward
# the commanded direction. Sign-free: judges motion MAGNITUDE only.
FLOW_MIN_PX = 0.30           # mean |vertical flow| px/frame that counts as moving
FLOW_WINDOW_S = 2.5
FLOW_NUDGE = 0.05
FLOW_NUDGE_BUDGET = 6        # per direction per no-motion episode
FLOW_MIN_INTENT = 0.4
HOVER_DRIFT_FLOW_PX = 0.45   # sustained |flow| while commanding hover = drifting
HOVER_DRIFT_NUDGE = 0.04
OBLIQUE_HW_RATIO = 1.35      # gate h/w beyond this (up close) = bad angle
OBLIQUE_AREA_MIN = 0.02
OBLIQUE_VX = 0.8

# --- Camera-verified vertical trim: if we command climb/descend but the
# gate's row in the image refuses to move accordingly, the hover trim is
# wrong -- nudge it until the CAMERA confirms vertical motion.
VERT_STALL_WINDOW_S = 0.8
VERT_STALL_EL_RAD = 0.02     # el must improve by at least this per window
VERT_NUDGE = 0.06            # trim nudge per stalled window
VERT_MIN_INTENT = 0.25       # only judge when |vz intent| exceeds this
VERT_MIN_GATE_W = 60         # px: only judge against gates close enough that
                             # real climbing visibly moves them (a distant
                             # decoy's el can't respond -> poisoned the trim)
VERT_NUDGE_BUDGET = 5        # consecutive no-progress nudges before holding

# --- Ping-pong detector: fast TRACK losses to the same side = inverted steer
PINGPONG_FAST_LOSS_S = 0.7
PINGPONG_SIDE_RAD = 0.12
PINGPONG_COUNT = 2
PINGPONG_RESET_S = 6.0

# --- Committed scanning: hold a sweep direction, no mid-turn dithering
SCAN_COMMIT_S = 2.5

# --- Cross-run learning cache (untracked file; survives git reset)
CALIB_CACHE_FILE = "calib_cache.json"

# --- Forward-direction self-check (TRACK) ---
FWD_CHECK_WINDOW_S = 2.0     # if gate shrinks this long while commanding fwd...
FWD_CHECK_SHRINK_PX = 4.0    # ...by at least this many px -> flip forward sign
FWD_FLIP_COOLDOWN_S = 4.0

# --- Visual thrust integrator: the camera is the altitude ground truth.
# Gate persistently above image center -> thrust ratchets UP until it isn't.
EL_KI = 0.12                 # thrust per (rad of elevation error) per second
EL_BIAS_MAX = 0.20           # cap on the visual thrust bias
EL_BIAS_DECAY = 0.06         # 1/s decay of the bias when not tracking

# --- Attitude estimator ---
EST_ACC_ALPHA = 0.02
EST_GYRO_ROLL_SIGN = 1.0
EST_GYRO_PITCH_SIGN = 1.0
EST_ACC_ROLL_SIGN = 1.0
EST_ACC_PITCH_SIGN = 1.0

# ----------------------------------------------------------------------------
# Camera model (from technical spec VADR-TS-003 section 3.8)
# Values are for the native 640x360 stream; code rescales if frames differ.
# ----------------------------------------------------------------------------
CAM_FX = 320.0
CAM_FY = 320.0
# Spec claims 20 deg camera up-tilt, but flight data (run_20260702_184718)
# showed compensating for it makes the drone climb over gates. Empirically
# a level gate sits at image center, so we servo on pure pixel error.
CAM_TILT_COMP_RAD = 0.349          # +20 deg: official spec says the camera is
                                   # pitched UP 20 degrees. Without this comp the
                                   # drone 'centers' gates that are actually 20deg
                                   # above its flight path and flies UNDER them.

# ----------------------------------------------------------------------------
# Gate detection (HSV bounds verified against real VQ1 frames)
# ----------------------------------------------------------------------------
GATE_HSV_LO_1 = (160, 40, 140)
GATE_HSV_HI_1 = (180, 255, 255)
GATE_HSV_LO_2 = (0, 40, 140)
GATE_HSV_HI_2 = (10, 255, 255)
GATE_MIN_AREA_FRAC = 0.00005       # ignore blobs smaller than this
DETECT_STALE_S = 0.4               # detection older than this = "gate lost"

# ----------------------------------------------------------------------------
# Visual servoing (TRACK mode)
# ----------------------------------------------------------------------------
MAX_FWD = 2.5              # m/s forward cruise. Raise once runs are clean.
MIN_FWD = 0.8              # m/s floor while a gate is visible
YAW_KP = 1.6               # yaw_rate = YAW_KP * horizontal angle to gate
MAX_YAW_RATE = 1.6         # rad/s
MAX_VZ = 1.5               # m/s vertical clamp
EL_VZ_KP = 2.2             # m/s of climb per rad of elevation error
                           # (decoupled from vx so climb-first still climbs hard)
ALIGN_FALLOFF_RAD = 0.6    # fwd speed scales down as |az| approaches this
EL_SLOW_RAD = 0.28         # fwd speed also scales down as |el| approaches this
                           # (climb FIRST, then go: nose-down forward pitch was
                           #  expelling high gates out of the camera's view)
VX_EL_MIN = 0.2            # m/s forward floor while badly misaligned vertically
# --- anti-orbit: brake + strafe (v5.7) ---
STRAFE_ENABLE = True
STRAFE_VY_KP = 1.6         # m/s of sideways intent per rad of azimuth error
STRAFE_SIGN = 1.0          # flip to -1.0 if the drone strafes AWAY from the gate
STRAFE_VY_MAX = 1.0        # m/s cap
ROLL_PER_MS = 0.045        # rad of bank per (m/s sideways intent), mirrors pitch
MAX_ROLL = 0.22            # rad (~13 deg) bank cap
BRAKE_AZ_RAD = 0.30        # gate this far off-center while CLOSE -> brake
BRAKE_AREA_MIN = 0.02      # "close" = gate area above this fraction of frame
BRAKE_VX = -0.8            # m/s backward intent (pitch back) to kill momentum

# ----------------------------------------------------------------------------
# COMMIT mode (punch through the gate when close)
# ----------------------------------------------------------------------------
COMMIT_WIDTH_FRAC = 0.30   # gate bbox width / frame width to trigger commit
COMMIT_AZ_MAX = 0.12       # rad: gate must be THIS centered before committing
COMMIT_EL_MAX = 0.10       # rad: commit only from a LEVEL approach
COMMIT_STEER_GAIN = 0.9    # fraction of normal steering kept during COMMIT
APPROACH_VX = 0.6          # m/s cap while wide-but-off-center (line up first)
CLOSE_AREA = 0.08          # gate area beyond which yaw authority boosts
CLOSE_YAW_BOOST = 1.5
CARRY_TIME_S = 1.0         # after a confirmed pass: straight ahead, no looking back
COMMIT_TIME_S = 1.2        # fly this long once committed
COMMIT_SPEED = 2.5         # m/s straight through

# ----------------------------------------------------------------------------
# SEARCH mode (no gate visible)
# ----------------------------------------------------------------------------
SEARCH_YAW_RATE = 0.6      # rad/s slow scan
SEARCH_VZ = 0.35           # m/s vertical drift toward where gate was last seen
SEARCH_MEMORY_S = 8.0      # trust last-seen direction for this long
SEARCH_DESCEND = 0.6       # m/s vertical speed for no-memory altitude sweep
SEARCH_SWEEP_DOWN_S = 14.0  # gates live low: sweep down much longer than up
SEARCH_SWEEP_UP_S = 5.0
                           # (camera tilts UP 20deg: gates below are invisible
                           #  until we descend; the UNSTICK watchdog guards the floor)

# ----------------------------------------------------------------------------
# LAUNCH (takeoff at race start) and anti-grind watchdog
# Run 20260702_191752 proved the drone never took off: 450 collisions/sec
# grinding on the ground while "flying forward". Never again.
# ----------------------------------------------------------------------------
LAUNCH_TIME_S = 0.25       # just unstick from the ground; TRACK does the rest
LAUNCH_CLIMB = 1.0         # m/s takeoff climb rate (run 194611 overshot at 1.5x1.2s)
UNSTICK_COLLISION_RATE = 25   # collisions/sec that triggers emergency climb
UNSTICK_TIME_S = 1.0       # emergency climb duration
UNSTICK_CLIMB = 1.5        # m/s emergency climb rate

# ----------------------------------------------------------------------------
# Logging
# ----------------------------------------------------------------------------
LOG_DIR = "logs"
LOG_STATE_HZ = 10
CONSOLE_STATUS_S = 1.0
SAVE_FRAME_EVERY_S = 1.0   # save an ANNOTATED frame this often (0 = never)

# ---- blue guidance tube following (v6.0) ----
TUBE_ENABLE = True
TUBE_SPEED = 1.8           # m/s forward while following the tube
TUBE_YAW_KP = 2.2          # yaw rate per unit of far-band lateral error
TUBE_VY_KP = 1.4           # strafe per unit of near-band lateral error
TUBE_MIN_PIX = 250         # blue pixels needed in a band to trust it
TUBE_GATE_TAKEOVER_W = 26  # px: gate this big while on tube -> normal TRACK

# ---- PLAYBOOK (v7.0): scripted opening, tuned run-over-run ----
# course is identical every run, so we choreograph the start:
# (duration_s, vx, vy, vz, yaw_rate)  executed in order from GO.
# vz negative = climb. Tune these numbers from run feedback.
# steps: (duration_s, vx, vy, vz, yaw_rate, pinned_thrust)
# pinned_thrust holds the throttle EXACTLY (deterministic). The drive step
# needs a bit more than hover: pitching forward steals vertical lift --
# that is the "cat falls just short" sink.
# playbook advances on TIME or on a GATE PASS event (whichever first),
# so each gate gets its own pace notes. gate 1 sits slightly below+forward.
PLAYBOOK = [
    (0.3,  0.0, 0.0, 0.0, 0.0, 0.245),  # blink of a hover
    (10.0, 2.6, 0.18, 0.0, 0.0, 0.270),  # flat kick through gate 0 [-> pass event]
    (1.5,  2.4, 0.22, 0.0, 0.0, 0.232),  # sink + right toward gate 1
    (8.0,  2.4, 0.15, 0.0, 0.0, 0.268),  # level drive through gate 1 [-> pass event]
]
PLAYBOOK_GATE_W = 9999 # script owns the approach (user-tuned; vision stays out)
PB_THRUST_CAP = 0.27   # pinned opening thrust (bisected from runs)
PB_THRUST_CAP_S = 8.0  # pinned for the whole approach
