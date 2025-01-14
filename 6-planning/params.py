TARGET_SPEED = 10.0  # target speed [m/s]

MAX_SPEED = 25.0  # maximum speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 2.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 5.0  # maximum road width [m]
D_ROAD_W = 0.5  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAX_T = 5.0  # max prediction time [s]
MIN_T = 4.5  # min prediction time [s]
D_T_S = 0.5  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 3.0  # robot radius [m]

# Cost weights
K_J = 0.1
K_T = 1.0
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

# Figures rendering
FIGS_PATH = "results/figures/ex3/speed-max/mpc-linear-st"
FIGS_NAMES = [
    "1-traj.png",
    "2-heading.png",
    "3-long-vel.png",
    "4-lat-vel.png",
    "5-yaw-rate.png",
    "6-front-slip.png",
    "7-rear-slip.png",
    "8-steer.png",
    "9-side-slip.png",
    "10-lateral-force.png",
    "11-lateral-error.png",
    "12-velocity-error.png",
    "13-long-accel.png"
]
FIGS_IDX = 0