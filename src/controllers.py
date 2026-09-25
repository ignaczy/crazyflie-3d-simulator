import numpy as np

class BaseController:
    """Common interface for all control algorithms with full feedforward (v_ff, a_ff)."""
    def compute(self, target: float, current: float, dt: float, v_ff: float = 0.0, a_ff: float = 0.0) -> float:
        raise NotImplementedError
    def reset(self):
        raise NotImplementedError

class PIDController(BaseController):
    def __init__(self, Kp=2.0, Ki=0.0, Kd=0.5, limit=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current, dt, v_ff=0.0, a_ff=0.0):
        if dt <= 0:
            return 0.0
        error = target - current
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.limit, self.limit)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        # Full feedforward: v_ff [m/s] and velocity increment resulting from a_ff (a_ff * dt)
        out = self.Kp * error + self.Ki * self.integral + self.Kd * derivative + v_ff + a_ff * dt
        return np.clip(out, -self.limit, self.limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class ADRCController(BaseController):
    """
    Linear 2nd-order ADRC controller with an ESO observer 
    and full velocity and acceleration feedforward.
    """
    def __init__(self, b0=10.0, wc=15.0, wo=30.0, limit=1.0):
        self.b0 = b0
        self.wc = wc
        self.wo = wo
        self.limit = limit
        
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self._last_u = 0.0
        self._initialized = False

        self.update_gains()

    def update_gains(self):
        self.beta1 = 3.0 * self.wo
        self.beta2 = 3.0 * (self.wo ** 2)
        self.beta3 = self.wo ** 3

        self.kp = self.wc ** 2
        self.kd = 2.0 * self.wc

    def compute(self, target, current, dt, v_ff=0.0, a_ff=0.0):
        if dt <= 0:
            return 0.0
        
        if not self._initialized:
            self.z1 = current
            self.z2 = 0.0
            self.z3 = 0.0
            self._initialized = True

        self.update_gains()

        sub_steps = 10
        dt_sub = dt / sub_steps

        for _ in range(sub_steps):
            e = self.z1 - current
            z1_dot = self.z2 - self.beta1 * e
            z2_dot = self.z3 + self.b0 * self._last_u - self.beta2 * e
            z3_dot = -self.beta3 * e
            
            self.z1 += z1_dot * dt_sub
            self.z2 += z2_dot * dt_sub
            self.z3 += z3_dot * dt_sub

        error = target - self.z1
        
        # Control law accounting for a_ff in the acceleration layer and v_ff
        u_unclipped = (self.kp * error - self.kd * (self.z2 - v_ff) - self.z3 + a_ff) / self.b0
        
        u = np.clip(u_unclipped, -self.limit, self.limit)
        self._last_u = u
        return u

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self._last_u = 0.0
        self._initialized = False


class MPCController:
    def __init__(self, horizon=10, Q_pos=10.0, R_u=1.0, limit=1.0):
        self.N = int(horizon)       # Prediction horizon
        self.Q = float(Q_pos)       # Position error weight
        self.R = float(R_u)         # Control effort weight
        self.limit = float(limit)   # Output velocity limit
        self.last_val = 0.0
        self.reset()

    def reset(self):
        self.last_val = 0.0
        self.last_u = 0.0

    def compute(self, setpoint, pv, dt, v_ff=0.0, a_ff=0.0):
        """
        MPC with full reference trajectory prediction based on position, v_ff, and a_ff.
        """
        if dt <= 0:
            dt = 0.016
            
        vel = (pv - self.last_val) / dt
        self.last_val = pv

        x0 = np.array([pv, vel], dtype=float)

        A = np.array([[1.0, dt], 
                      [0.0, 1.0]], dtype=float)
        B = np.array([[0.5 * dt**2], 
                      [dt]], dtype=float)

        S_x = np.zeros((self.N, 2), dtype=float)
        S_u = np.zeros((self.N, self.N), dtype=float)

        A_power = np.eye(2, dtype=float)
        for i in range(self.N):
            A_power = A_power @ A
            S_x[i, :] = A_power[0, :]  
            
            for j in range(i + 1):
                mat_pow = np.linalg.matrix_power(A, i - j)
                S_u[i, j] = (mat_pow @ B)[0, 0]

        # Building the reference trajectory incorporating kinematics: p(t) = p_0 + v_ff*t + 0.5*a_ff*t^2
        R_ref = np.zeros(self.N, dtype=float)
        for i in range(self.N):
            t_pred = (i + 1) * dt
            R_ref[i] = setpoint + v_ff * t_pred + 0.5 * a_ff * (t_pred ** 2)

        Q_bar = self.Q * np.eye(self.N, dtype=float)
        R_bar = self.R * np.eye(self.N, dtype=float)

        try:
            term = S_u.T @ Q_bar @ S_u + R_bar
            M_inv = np.linalg.inv(term)
            
            error_term = S_x @ x0 - R_ref
            U_opt = - M_inv @ (S_u.T @ Q_bar @ error_term)
            
            # Base MPC output + feedforward velocity
            v_des = U_opt[0] + v_ff
            
        except np.linalg.LinAlgError:
            v_des = v_ff

        v_des = float(np.clip(v_des, -self.limit, self.limit))
        self.last_u = v_des
        
        return v_des