import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simulation import Simulation

# Sinusoidal input if True, constant otherwise
SIN_INPUT: bool = True

TIME_QUANTA = 0.001
ACC_X = 1.0
STEER_ANGLE = 0.055
SIM_TIME = 5
INIT_LONG_VEL = 27

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
    plt.show()

def plot_trajectory(x_vals, y_vals, labels):
    """ Plot 2D trajectory (x vs y) for all simulation configurations. """
    plt.figure(figsize=(10, 6))
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

def run_simulation(ax, steer, dt, integrator, model, steps=500):
    """ Run a simulation with the given parameters and return all states. """
    # Vehicle parameters
    lf = 1.156          # Distance from COG to front axle (m)
    lr = 1.42           # Distance from COG to rear axle (m)
    mass = 1200         # Vehicle mass (kg)
    Iz = 1792           # Yaw moment of inertia (kg*m^2)

    # Initialize the simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model, init_vx=INIT_LONG_VEL)

    # Storage for state variables and slip angles
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals = [], [], [], [], [], []
    alpha_f_vals, alpha_r_vals, beta_vals = [], [], []  # Slip angles
    delta_vals = []  # Steering angles
    fy_vals, fy_slips = [], []

    # Max steer and frequency for sinusoidal steer commands
    steer_max = 0.1
    frequency = 0.5

    for step in range(steps):
        # Make one step simulation via model integration
        # Calculate sinusoidal steering angle
        time = step * dt

        if SIN_INPUT:
            steer = steer_max * np.sin(2 * np.pi * frequency * time)  # Sinusoidal steering angle
        else:
            steer = STEER_ANGLE

        sim.integrate(ax, steer)

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
        delta_vals.append(steer)
        beta_vals.append(beta)
        fy_vals.append(sim.Fyf)
        fy_slips.append(sim.alpha_f)

        
    order = np.argsort(fy_slips)
    fy_vals, fy_slips = np.array(fy_vals)[order], np.array(fy_slips)[order]
    return x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals, delta_vals, beta_vals, fy_vals, fy_slips

def main():
    # Simulation parameters
    dt = TIME_QUANTA        # Time step (s)
    ax = ACC_X              # Constant longitudinal acceleration (m/s^2)
    steer = STEER_ANGLE     # Constant steering angle (rad)
    sim_time = SIM_TIME     # Simulation duration in seconds
    steps = int(sim_time / dt)  # Simulation steps (30 seconds)

    # List of configurations
    configs = [
        ("rk4", "kinematic"),
        ("rk4", "linear"),
        ("rk4", "nonlinear"),
        ("euler", "kinematic"),
        ("euler", "linear"),
        ("euler", "nonlinear")
    ]

    # Run each simulation and store the results
    all_results = []
    labels = []
    for integrator, model in configs:
        results = run_simulation(ax, steer, dt, integrator, model, steps)
        all_results.append(results)
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

    # Plot comparisons for each state variable
    plot_trajectory(x_results, y_results, labels)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)")
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)")
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)")
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)")
    plot_comparison(alpha_f_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front")
    plot_comparison(alpha_r_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear")
    plot_comparison(delta_results, labels, "Steering Angle Comparison", "Time Step", "Steering Angle (rad)")
    plot_comparison(beta_results, labels, "Side Slip Angle Comparison", "Time Step", "Side Slip Angle (rad)")
    plot_comparison(fy_results, labels, "Lateral Tire Force Comparison", "Slip Angle", "Lateral Tire Force (N)", fy_slips)

if __name__ == "__main__":
    main()
