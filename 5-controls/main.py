import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simulation import Simulation
import pid
import purepursuit
import stanley
from mpc import *
import cubic_spline_planner
import math
from enum import Enum, auto
from os.path import join
from os import getcwd

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

class Controller(Enum):
    PURE_PURSUIT = auto()
    STANLEY = auto()
    MPC = auto()
    NONE = auto()

# Simulation parameters
selected_controller: Controller = Controller.MPC
dt = 0.05             # Time step (s)
ax = 0.0              # Constant longitudinal acceleration (m/s^2)
vx = 0.0              # Initial longitudinal velocity
steer = 0.0           # Constant steering angle (rad)

# Control references
target_speed = 29.0

# Vehicle parameters
lf = 1.156          # Distance from COG to front axle (m)
lr = 1.42           # Distance from COG to rear axle (m)
wheelbase = lf + lr
mass = 1200         # Vehicle mass (kg)
Iz = 1792           # Yaw moment of inertia (kg*m^2)
max_steer = 3.14  # Maximum steering angle in radians

# Create instance of PID for Longitudinal Control
long_control_pid = pid.PIDController(kp=2, ki=0.04, kd=0.1, output_limits=(-2, 2), anti_windup_gain=0.001)

# Create instance of PurePursuit, Stanley and MPC for Lateral Control
k_v_pp = 0.05  # Speed proportional gain for Pure Pursuit
k_c_pp = 0.0001  # Curvature proportional gain for Pure Pursuit
base_look_ahead = 0.5  # Minimum look-ahead distance for Pure Pursuit
min_look_ahead = lf
pp_controller = purepursuit.PurePursuitController(wheelbase, max_steer)

k_stanley = 6.0  # Gain for cross-track error for Stanley
stanley_controller = stanley.StanleyController(k_stanley, lf, max_steer, 1.3, 2.0)

def load_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        line = file.readline()
        xs.append( float(line.split(",")[0]) )
        ys.append( float(line.split(",")[1]) )
    return xs, ys

# Load path and create a spline
xs, ys = load_path("oval_trj.txt")
path_spline = cubic_spline_planner.Spline2D(xs, ys)

def point_transform(trg, pose, yaw):

    local_trg = [trg[0] - pose[0], trg[1] - pose[1]]

    return local_trg

def save_figure(fig):
    global FIGS_IDX
    
    with open(join(getcwd(), FIGS_PATH, FIGS_NAMES[FIGS_IDX]), "wb") as fig_file:
        fig.savefig(fig_file)
    
    FIGS_IDX += 1

def plot_comparison(results, labels, title, xlabel, ylabel, x_vals=None):
    """ Plot comparison of results for a specific state variable. """

    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        if x_vals: plt.plot(x_vals[i], result, label=labels[i])
        else: plt.plot(result, label=labels[i])
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    fig = plt.gcf()
    plt.show()
    save_figure(fig)


def plot_trajectory(x_vals, y_vals, labels, path_spline):
    """ Plot 2D trajectory (x vs y) for all simulation configurations and path_spline trajectory. """
    plt.figure(figsize=(10, 6))
    
    # Plot the simulation trajectories
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
    
    # Plot the path_spline trajectory
    spline_x = [path_spline.calc_position(s)[0] for s in np.linspace(0, path_spline.s[-1], 1000)]
    spline_y = [path_spline.calc_position(s)[1] for s in np.linspace(0, path_spline.s[-1], 1000)]
    plt.plot(spline_x, spline_y, label="Path Spline", linestyle="--", color="red")
    
    # Customize plot
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    fig = plt.gcf()
    plt.show()
    save_figure(fig)

def run_simulation(ax, steer, dt, integrator, model):
    """ Run a simulation with the given parameters and return all states. """

    # Initialize the simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model, init_vx=vx)
    start_x = sim.x
    first_half_turn = True

    # Storage for state variables and slip angles
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals = [], [], [], [], [], []
    alpha_f_vals, alpha_r_vals, beta_vals = [], [], []  # Slip angles
    delta_vals = []  # Steering angles
    fy_vals, fy_slips = [], []
    lat_errors, vel_errors, long_acc = [], [], []

    if selected_controller == Controller.MPC:
        casadi_model(sim)

    total_error = 0
    step = 0

    while sim.x <= start_x+1 or first_half_turn:
        if sim.x < start_x-1: first_half_turn = False
        step+=1
    
        # Print time
        print("Time:", step*dt)

        # Calculate ax to track speed
        ax = long_control_pid.compute(target_speed, sim.vx, dt) # Exercise 1

        # Update actual frenet-frame position in the spline
        # aka longitudinal position and actual lateral error
        actual_position = sim.x, sim.y
        actual_pose = sim.x, sim.y, sim.theta
        path_spline.update_current_s(actual_position)

        # get actual position projected on the path/spline
        position_projected = path_spline.calc_position(path_spline.cur_s)
        prj = [ position_projected[0], position_projected[1] ]
        local_error = point_transform(prj, actual_position, sim.theta)
        lat_error = sqrt(local_error[0]*local_error[0] + local_error[1]*local_error[1])
        total_error += lat_error

        vel_error = sim.vx - target_speed

        if(abs(local_error[1]) > 1.0):
            print("Lateral error is higher than 1.0... ending the simulation")
            print("Lateral error: ", local_error[1])
            break

        # Calculate steer to track path
        match selected_controller:
            case Controller.PURE_PURSUIT:
                # Compute the look-ahead distance
                Lf = base_look_ahead + k_v_pp * sim.vx + k_c_pp / max(1e-2, abs(path_spline.calc_curvature(path_spline.cur_s)))
                Lf = max(Lf, min_look_ahead)
                s_pos = path_spline.cur_s + Lf

                # get target pose
                trg = path_spline.calc_position(s_pos)
                trg = [ trg[0], trg[1] ]
                # Adjust CoG position to the rear axle position for PP
                pp_position = actual_position[0] - lr * math.cos(sim.theta), actual_position[1] - lr * math.sin(sim.theta)
                loc_trg = point_transform(trg, pp_position, sim.theta)
                steer = pp_controller.compute_steering_angle(loc_trg, sim.theta, Lf)
                
            case Controller.STANLEY:
                stanley_target = position_projected[0], position_projected[1], path_spline.calc_yaw(path_spline.cur_s)
                steer = stanley_controller.compute_steering_angle(actual_pose, stanley_target, sim.vx)

            case Controller.MPC:
                # get future horizon targets pose
                targets = []
                s_pos = path_spline.cur_s
                for _ in range(N):
                    trg = path_spline.calc_position(s_pos)
                    trg_theta = path_spline.calc_yaw(s_pos)
                    trg = [trg[0], trg[1], trg_theta]
                    targets.append(trg)
                    s_pos += sim.vx*dt
                steer = opt_step(targets, sim)
            
            case Controller.NONE:
                steer = 0

        # Make one step simulation via model integration
        sim.integrate(ax, float(steer))
        beta = np.arctan(sim.vy/sim.vx)  # Side slip angle
        
        # Append each state to corresponding list
        x_vals.append(sim.x)
        y_vals.append(sim.y)
        theta_vals.append(sim.theta)
        vx_vals.append(sim.vx)
        vy_vals.append(sim.vy)
        r_vals.append(sim.r)
        alpha_f_vals.append(sim.alpha_f)
        alpha_r_vals.append(sim.alpha_r)
        delta_vals.append(float(steer))
        beta_vals.append(beta)
        fy_vals.append(sim.Fyf)
        fy_slips.append(sim.alpha_f)
        lat_errors.append(lat_error)
        vel_errors.append(vel_error)
        long_acc.append(ax)


    print(f"{total_error=}")
    order = np.argsort(fy_slips)
    fy_vals, fy_slips = np.array(fy_vals)[order], np.array(fy_slips)[order]
    return x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals, delta_vals, beta_vals, fy_vals, fy_slips, lat_errors, vel_errors, long_acc


def main():

    # List of configurations
    configs = [
        ("rk4", "nonlinear"),
    ]

    # Run each simulation and store the results
    all_results = []
    actual_state = []
    labels = []
    for integrator, model in configs:
        actual_state = run_simulation(ax, steer, dt, integrator, model)
        all_results.append(actual_state)
        labels.append(f"{integrator.capitalize()} - {model.capitalize()}")

    # Separate each state for plotting
    x_results = [result[0] for result in all_results]
    y_results = [result[1] for result in all_results]
    theta_results = [result[2] for result in all_results]
    vx_results = [result[3] for result in all_results]
    vy_results = [result[4] for result in all_results]
    r_results = [result[5] for result in all_results]
    alpha_f_results = [result[6] for result in all_results]
    alpha_r_results = [result[7] for result in all_results]
    delta_results = [result[8] for result in all_results]
    beta_results = [result[9] for result in all_results]
    
    fy_results = [result[10] for result in all_results]
    fy_slips = [result[11] for result in all_results]

    lat_errors = [result[12] for result in all_results]
    vel_errors = [result[13] for result in all_results]
    long_acc = [result[14] for result in all_results]


    # Plot comparisons for each state variable
    plot_trajectory(x_results, y_results, labels, path_spline)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)")
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_f_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_r_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")

    plot_comparison(delta_results, labels, "Steering Angle Comparison", "Time Step", "Steering Angle (rad)")
    plot_comparison(beta_results, labels, "Side Slip Angle Comparison", "Time Step", "Side Slip Angle (rad)")
    plot_comparison(fy_results, labels, "Lateral Tire Force Comparison", "Slip Angle", "Lateral Tire Force (N)", fy_slips)

    plot_comparison(lat_errors, labels, "Lateral Error Comparison", "Time Step", "Lateral Error (m)")
    plot_comparison(vel_errors, labels, "Velocity Error Comparison", "Time Step", "Velocity Error (m/s)")
    plot_comparison(long_acc, labels, "Longitudinal Acceleration Comparison", "Time Step", "Longitudinal Acceleration (m/s^2)")


if __name__ == "__main__":
    main()
