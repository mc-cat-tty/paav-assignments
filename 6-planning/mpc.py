from cubic_spline_planner import *
from casadi import *
from casadi.tools import *
from simulation import Simulation

# MPC time
T =  1.0 # Horizon length in seconds
dt = 0.05 # Horizon timesteps
N = int(T/dt) # Horizon total points
F = None

max_steer = 3.14  # Maximum steering angle in radians
min_steer = -3.14  # Minimum steering angle in radians

def casadi_model(sim: Simulation):
    global F

    # Control
    # Create 1r-1c matrix containing control inputs. 
    # Set steer as the only input element
    u = MX.sym("u", 1)
    steer = u[0]

    # State
    x = MX.sym("x", 6)
    sx, sy, yaw, speed_long, speed_lat, yaw_rate = vertsplit(x)

    Fz = sim.mass * 9.81      # Normal force

    # NON LINEAR ST MODEL
    # alpha_f = steer - np.arctan((speed_lat + sim.l_f * yaw_rate) / speed_long)
    # alpha_r = - np.arctan((speed_lat - sim.l_r * yaw_rate) / speed_long)
    # -----------------

    # LINEAR ST MODEL
    alpha_f = steer - (speed_lat + sim.l_f * yaw_rate) / speed_long
    alpha_r = - (speed_lat - sim.l_r * yaw_rate) / speed_long
    # -----------------
    
    Fzf = Fz * sim.l_r/sim.l_wb
    Fzr = Fz * sim.l_f/sim.l_wb

    # Front and rear lateral forces
    lateral_force_f = lambda Fz, alpha: Fz * sim.D * np.sin(sim.C * np.arctan(sim.B * alpha - sim.E * (sim.B * alpha - np.arctan(sim.B * alpha))))

    # NON LINEAR ST MODEL
    # Fyf = lateral_force_f(Fzf, alpha_f)
    # Fyr = lateral_force_f(Fzr, alpha_r)
    # -----------------

    # LINEAR ST MODEL
    Fyf = alpha_f * sim.Cf * Fzf
    Fyr = alpha_r * sim.Cr * Fzr
    # -----------------

    # ODE right hand side
    sxdot    = speed_long*cos(yaw) - speed_lat*sin(yaw)  # vx
    sydot    = speed_long*sin(yaw) + speed_lat*cos(yaw)  # vy
    yawdot   = yaw_rate
    speeddot_long = 0  # Since it acts just as lateral controller, long control provided by someone else
    speeddot_lat = (Fyr + Fyf * cos(steer)) / sim.mass - speed_long * yaw_rate
    yaw_rate_dot = (Fyf * sim.l_f * cos(steer) - Fyr * sim.l_r) / sim.I_z

    # Concatenate vertically the expressions creating a row vector
    xdot = vertcat(sxdot, sydot, yawdot, speeddot_long, speeddot_lat, yaw_rate_dot)

    # ODE right hand side function
    # as input are used the state and control inputs,
    # and as output the row vector containing the model expressions
    f = Function('f', [x,u],[xdot])

    # Integrate the step with Explicit Euler
    IN = 1
    xj = x
    for _ in range(IN):
        fj = f(xj,u)
        xj += dt*fj/IN

    # Discrete time dynamics function
    F = Function('F', [x,u],[xj])

def opt_step(target, state):
    global F

    # Control for all segments
    nu = N #number of states
    Us = MX.sym("U",nu) #steer control input

    # Initial conditions
    x0 = [state.x, state.y, state.theta, state.vx, state.vy, state.r]
    X0 = MX(x0) # vector containing the initial state

    J = 0 # objective function that should be minimized by the nlp solver

    # build graph
    X=X0
    G = None
    lbg = []
    ubg = []

    # For every temporal step
    for k in range(nu):
        X = F(X, vertcat(Us[k]))  # Integrate the step
        gain_mult = 1

        # Give more importance to the last step using a bigger gain
        if(k == nu-1):
            gain_mult=2 # You can use this multiplier as terminal cost

        J += 100.0*gain_mult*(X[0]-target[k][0])**2  # x error cost 
        J += 100.0*gain_mult*(X[1]-target[k][1])**2  # y error cost
        J += 10.00*gain_mult*(X[2]-target[k][2])**2  # heading error cost
    
    # G = X[index] #if you want to set a state to constrain in arg["lbg"] and arg["ubg"]. It can be ignored


    # Objective function and constraints
    J += mtimes(Us.T,Us) * (1000.0 + 100 * state.vx)    # Speed-dependent

    # NLP
    nlp = {'x':vertcat(Us), 'f':J}
    
    # Allocate an NLP solver
    opts = {
    "ipopt.tol": 1e-3,
    "ipopt.acceptable_tol": 1e-2,
    "ipopt.acceptable_iter": 5,
    "ipopt.max_iter": 20,
    "ipopt.print_level": 1,
    "expand": True,
    }

    solver = nlpsol("solver", "ipopt", nlp, opts)
    arg = {}

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(min_steer*np.ones(nu)) # lower bound for steer
    arg["ubx"] =  vertcat(max_steer*np.ones(nu)) # upper bound for steer
    arg["x0"] =    0.0 # first guess 

    #These can be ingnored 
    # # Bounds on g
    # arg["lbg"] = lower_bound on a state
    # arg["ubg"] = inf

    # Solve the problem
    res = solver(**arg)
    #print "f:", res["f"]
    ctrls = reshape(res["x"], (nu,1)).T #reshape to have a row for each step

    return ctrls[0][0]