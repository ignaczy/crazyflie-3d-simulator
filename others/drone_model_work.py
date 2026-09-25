import numpy as np
import math

class InternalPID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measured, dt):
        if dt <= 0:
            return 0.0
        error = setpoint - measured
        self.integral = float(np.clip(self.integral + error * dt, -self.limit, self.limit))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return float(np.clip(out, -self.limit, self.limit))

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class Crazyflie3D:
    def __init__(self):
        # Stan drona [x, y, z], prędkość [vx, vy, vz], kąty [roll, pitch, yaw], prędkości kątowe [p, q, r]
        self.pos = np.array([0.0, 0.0, 1.0], dtype=float)
        self.vel = np.array([0.0, 0.0, 0.0], dtype=float)
        self.rot = np.array([0.0, 0.0, 0.0], dtype=float) 
        self.omega = np.array([0.0, 0.0, 0.0], dtype=float)
        
        self.prop_angle = 0.0  
        self.motor_rpms = np.zeros(4, dtype=float)
        self.last_v_des = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Parametry fizyczne
        self.mass = 0.027
        self.g = 9.81
        self.hover_rpm = 14500.0  

    def reset_state(self):
        self.pos = np.array([0.0, 0.0, 1.0], dtype=float)
        self.vel.fill(0)
        self.rot.fill(0)
        self.omega.fill(0)
        self.motor_rpms.fill(0)
        self.last_v_des.fill(0)

    def step(self, target_pos, target_yaw, dt, pids, v_ff=None):
        pid_x, pid_y, pid_z = pids
        if dt <= 0:
            return

        # 1. Zewnętrzna pętla pozycji -> Prędkości zadane
        v_x_des = np.clip(pid_x.compute(target_pos[0], self.pos[0], dt), -1.5, 1.5)
        v_y_des = np.clip(pid_y.compute(target_pos[1], self.pos[1], dt), -1.5, 1.5)
        v_z_des = np.clip(pid_z.compute(target_pos[2], self.pos[2], dt), -1.5, 1.5)
        self.last_v_des = np.array([v_x_des, v_y_des, v_z_des])

        # 2. Sprzężenie translacji z orientacją (przeliczenie uchybu prędkości na żądane kąty)
        err_vx = v_x_des - self.vel[0]
        err_vy = v_y_des - self.vel[1]
        err_vz = v_z_des - self.vel[2]

        c_psi, s_psi = math.cos(self.rot[2]), math.sin(self.rot[2])
        body_err_x =  err_vx * c_psi + err_vy * s_psi
        body_err_y = -err_vx * s_psi + err_vy * c_psi

        max_tilt = math.radians(20.0)
        desired_pitch =  np.clip( 0.25 * body_err_x, -max_tilt, max_tilt)
        desired_roll  =  np.clip(-0.25 * body_err_y, -max_tilt, max_tilt)
        desired_yaw   =  target_yaw

        # 3. Płynne, stabilne śledzenie kątów (filtr inercyjny bez drgań)
        alpha = min(1.0, 35.0 * dt)
        roll_diff = desired_roll - self.rot[0]
        pitch_diff = desired_pitch - self.rot[1]
        yaw_diff = desired_yaw - self.rot[2]

        self.omega[0] = roll_diff / dt if dt > 0 else 0.0
        self.omega[1] = pitch_diff / dt if dt > 0 else 0.0
        self.omega[2] = yaw_diff / dt if dt > 0 else 0.0

        self.rot[0] += roll_diff * alpha
        self.rot[1] += pitch_diff * alpha
        self.rot[2] += yaw_diff * alpha

        # 4. Całkowity ciąg (Thrust)
        total_thrust = np.clip(self.mass * (self.g + err_vz * 4.0), 0.0, 2.0 * self.mass * self.g)

        # 5. Mikser RPM
        base_rpm = self.hover_rpm * math.sqrt(max(0.0, total_thrust / (self.mass * self.g)))
        d_roll = desired_roll * 3000.0
        d_pitch = desired_pitch * 3000.0
        d_yaw = 0.0

        self.motor_rpms[0] = np.clip(base_rpm + d_roll + d_pitch - d_yaw, 1000, 23000)
        self.motor_rpms[1] = np.clip(base_rpm - d_roll + d_pitch + d_yaw, 1000, 23000)
        self.motor_rpms[2] = np.clip(base_rpm - d_roll - d_pitch - d_yaw, 1000, 23000)
        self.motor_rpms[3] = np.clip(base_rpm + d_roll - d_pitch + d_yaw, 1000, 23000)

        # 6. Dynamika translacyjna z pełnym sprzężeniem osi (Macierz Rotacji)
        phi, theta, psi = self.rot[0], self.rot[1], self.rot[2]
        c_r, s_r = math.cos(phi), math.sin(phi)
        c_p, s_p = math.cos(theta), math.sin(theta)
        c_y, s_y = math.cos(psi), math.sin(psi)

        R_col3 = np.array([
            c_y * s_p * c_r + s_y * s_r,
            s_y * s_p * c_r - c_y * s_r,
            c_p * c_r
        ])

        accel_world = np.array([0.0, 0.0, -self.g]) + (total_thrust / self.mass) * R_col3
        self.vel += accel_world * dt
        self.pos += self.vel * dt

        # Warunek ziemi
        if self.pos[2] <= 0.0:
            self.pos[2] = 0.0
            self.vel.fill(0)
            self.omega.fill(0)
            self.rot.fill(0)

        self.prop_angle += np.mean(self.motor_rpms) * 0.1 * dt