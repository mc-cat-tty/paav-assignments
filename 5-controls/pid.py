class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None), anti_windup_gain=0.1):
        """
        
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            output_limits (tuple): Tuple (min, max) for output saturation limits.
            anti_windup_gain (float): Gain for anti-windup back-calculation.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.anti_windup_gain = anti_windup_gain
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_output = 0.0

    def reset(self):
        """
        Reset the controller state.
        """
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_output = 0.0

    def compute(self, setpoint, measured_value, dt):
        """
        Compute the PID output.
        
        Args:
            setpoint (float): Desired setpoint.
            measured_value (float): Measured process variable.
            dt (float): Time step.
        
        Returns:
            float: PID control output.
        """
        error = setpoint - measured_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Anti-windup: Adjust the integral term if it would lead to saturation
        # TO-DO: Complete, 
        lower_limit, upper_limit = self.output_limits
        # raw_output = proportional + integral  # Compute without derivative
        # if lower_limit is not None and raw_output < lower_limit:
        #     anti_windup_correction = (raw_output - lower_limit) / self.ki
        #     self.integral -= 0#TO-DO: Complete as in PID - Antiwindup slide
        #     integral = self.ki * self.integral
        # elif upper_limit is not None and raw_output > upper_limit:
        #     anti_windup_correction = (raw_output - upper_limit) / self.ki
        #     self.integral -= 0#TO-DO: Complete as in PID - Antiwindup slide
        #     integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate the total PID output
        output = proportional + integral + derivative
        
        # Apply saturation to the final output
        if lower_limit is not None:
            output = max(lower_limit, output)
        if upper_limit is not None:
            output = min(upper_limit, output)
        
        # Save state for next iteration
        self.previous_error = error
        self.previous_output = output
        
        return output
