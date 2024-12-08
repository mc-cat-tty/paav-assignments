import numpy as np

class Simulation:
    def __init__(self, lf, lr, mass, Iz, dt, integrator="euler", model="kinematic", init_vx = 0):
        """
        Initialize the simulation parameters.
        """
        self.l_f = lf                   # Distance to front axle (m)
        self.l_r = lr                   # Distance to rear axle (m)
        self.l_wb = lf + lr
        self.mass = mass                # Vehicle mass (kg)
        self.I_z = Iz                   # Yaw moment of inertia (kg*m^2)
        self.dt = dt                    # Time step (s)
        self.integrator = integrator    # Integrator choice
        self.model = model              # Model choice
        self.Fz = self.mass * 9.81      # Normal force
        
        # Aerodynamic and rolling resistance parameters
        self.rho = 1.225               # Air density (kg/m^3)
        self.C_d = 0.3                 # Drag coefficient (typical for cars)
        self.A = 2.2                   # Frontal area (m^2)
        self.C_rr = 0.015              # Rolling resistance coefficient

        # Initialize states
        self.x = 0                      # X position (m)
        self.y = 0                      # Y position (m)
        self.theta = 0                  # Heading angle (rad)
        self.vx = init_vx               # Longitudinal velocity (m/s)
        self.vy = 0                     # Lateral velocity (m/s)
        self.r = 0                      # Yaw rate (rad/s)
        self.alpha_f = 0                # Front sleep angle
        self.alpha_r = 0                # Rear sleep angle

        # Pacejka's Magic Formula coefficients
        self.B, self.C, self.D, self.E = 7.1433, 1.3507, 1.0489, -0.0074722
        self.B_f, self.C_f, self.D_f, self.E_f = self.B, self.C, self.D, self.E
        self.B_r, self.C_r, self.D_r, self.E_r = self.B, self.C, self.D, self.E
        
        self.Cf, self.Cr = self.B_f*self.C_f*self.D_f, self.B_r*self.C_r*self.D_r  # Cornering stiffness front/rear (N/rad)


    def kinematic_model(self, ax, delta):
        """ Kinematic single-track model equations of motion. """

        self.alpha_f = 0
        self.alpha_r = 0

        # Aerodynamic drag and rolling resistance forces
        v = np.linalg.norm((self.vx, self.vy))
        F_aero = (self.rho * self.C_d * self.A * v * v) / 2
        F_roll = self.C_rr * self.Fz/4

        dx = np.array([
            self.vx * np.cos(self.theta),
            self.vx * np.sin(self.theta),
            self.vx * np.tan(delta) / self.l_wb,
            ax * np.cos(self.theta) - (F_aero + F_roll) / self.mass,
            0,
            0
        ])

        return dx

    def linear_single_track_model(self, ax, delta):
        """ Linear single-track model with aerodynamic and rolling resistance. """
        
        # Tire slip angles
        self.alpha_f = delta - (self.vy + self.l_f * self.r) / self.vx
        self.alpha_r = - (self.vy - self.l_r * self.r) / self.vx

        # Front and rear vertical forces
        # geometrically distributed among front and rear axle
        Fzf = self.Fz * self.l_r/self.l_wb
        Fzr = self.Fz * self.l_f/self.l_wb

        # Front and rear lateral forces
        Fyf = self.alpha_f * self.Cf * Fzf
        Fyr = self.alpha_r * self.Cr * Fzr

        # Aerodynamic drag and rolling resistance forces
        v = np.linalg.norm((self.vx, self.vy))
        F_aero = (self.rho * self.C_d * self.A * v * v) / 2
        F_roll = self.C_rr * self.Fz

        # Dynamics equations
        _vx = self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta)
        _vy = self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta)
        _yaw_rate = self.r
        _dvx = ax + self.vy * self.r - (F_aero + F_roll + Fyf * np.sin(delta)) / self.mass
        _dvy = (Fyr + Fyf * np.cos(delta)) / self.mass - self.vx * self.r
        _dyaw_rate = (Fyf * self.l_f * np.cos(delta) - Fyr * self.l_r) / self.I_z

        dx = np.array([_vx, _vy, _yaw_rate, _dvx, _dvy, _dyaw_rate])
        
        return dx

    def nonlinear_single_track_model(self, ax, delta):
        """ Nonlinear single-track model with aerodynamic and rolling resistance. """
        
        # Tire slip angles
        self.alpha_f = delta - np.arctan((self.vy + self.l_f * self.r) / self.vx)
        self.alpha_r = - np.arctan((self.vy - self.l_r * self.r) / self.vx)

        # Front and rear vertical forces
        # geometrically distributed among front and rear axle
        Fzf = self.Fz * self.l_r/self.l_wb
        Fzr = self.Fz * self.l_f/self.l_wb

        # Front and rear lateral forces
        lateral_force_f = lambda Fz, alpha: Fz * self.D * np.sin(self.C * np.arctan(self.B * alpha - self.E * (self.B * alpha - np.arctan(self.B * alpha))))

        Fyf = lateral_force_f(Fzf, self.alpha_f)
        Fyr = lateral_force_f(Fzr, self.alpha_r)

        # Aerodynamic drag and rolling resistance forces
        v = np.linalg.norm((self.vx, self.vy))
        F_aero = (self.rho * self.C_d * self.A * v * v) / 2
        F_roll = self.C_rr * self.Fz

        # Dynamics equations
        _vx = self.vx * np.cos(self.theta) - self.vy * np.sin(self.theta)
        _vy = self.vx * np.sin(self.theta) + self.vy * np.cos(self.theta)
        _yaw_rate = self.r
        _dvx = ax + self.vy * self.r - (F_aero + F_roll + Fyf * np.sin(delta)) / self.mass
        _dvy = (Fyr + Fyf * np.cos(delta)) / self.mass - self.vx * self.r
        _dyaw_rate = (Fyf * self.l_f * np.cos(delta) - Fyr * self.l_r) / self.I_z

        dx = np.array([_vx, _vy, _yaw_rate, _dvx, _dvy, _dyaw_rate])
        return dx

    def integrate(self, ax, delta):
        """ Select the integrator method and apply it to update the state. """
        if self.integrator == "euler":
            self.euler_step(ax, delta)
        elif self.integrator == "rk4":
            self.rk4_step(ax, delta)

    def euler_step(self, ax, delta):
        """ Euler integration method. """
        dx = self.compute_dx(ax, delta)
        self.update_state(dx)

    def rk4_step(self, ax, delta):
        """ Runge-Kutta 4th order integration method. """
        k1 = self.compute_dx(ax, delta)
        self.update_state(k1, scale=0.5)
        
        k2 = self.compute_dx(ax, delta)
        self.update_state(k2, scale=0.5, revert=k1)
        
        k3 = self.compute_dx(ax, delta)
        self.update_state(k3, scale=1, revert=k2)

        k4 = self.compute_dx(ax, delta)
        
        # Combine k1, k2, k3, k4 for RK4 update
        dx = (k1 + 2*k2 + 2*k3 + k4) / 6
        self.update_state(dx)

    def compute_dx(self, ax, delta):
        """ Compute the state derivatives using the chosen model. """
        if self.model == "kinematic":
            return self.kinematic_model(ax, delta)
        elif self.model == "linear":
            return self.linear_single_track_model(ax, delta)
        elif self.model == "nonlinear":
            return self.nonlinear_single_track_model(ax, delta)

    def update_state(self, dx, scale=1, revert=None):
        """ Update state with scaled dx. Optionally revert previous state for RK4. """
        if revert is not None:
            self.x -= revert[0] * self.dt
            self.y -= revert[1] * self.dt
            self.theta -= revert[2] * self.dt
            self.vx -= revert[3] * self.dt
            self.vy -= revert[4] * self.dt
            self.r -= revert[5] * self.dt

        self.x += dx[0] * self.dt * scale
        self.y += dx[1] * self.dt * scale
        self.theta += dx[2] * self.dt * scale
        self.vx += dx[3] * self.dt * scale
        self.vy += dx[4] * self.dt * scale
        self.r += dx[5] * self.dt * scale
