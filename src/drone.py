import numpy as np
import math

class Crazyflie3D:
    def __init__(self):
        # Drone state [x, y, z], velocity [vx, vy, vz], angles [roll, pitch, yaw], angular velocities [p, q, r]
        self.pos = np.array([0.0, 0.0, 1.0], dtype=float)
        self.vel = np.array([0.0, 0.0, 0.0], dtype=float)
        self.rot = np.array([0.0, 0.0, 0.0], dtype=float) 
        self.omega = np.array([0.0, 0.0, 0.0], dtype=float)
        
        self.prop_angle = 0.0  
        # Constant propeller speed (all motors have constant RPM)
        self.motor_rpms = np.array([12000.0, 12000.0, 12000.0, 12000.0], dtype=float)
        self.last_v_des = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Physical parameters
        self.mass = 0.027
        self.g = 9.81
        self.hover_rpm = 12000.0  

    def reset_state(self):
        self.pos = np.array([0.0, 0.0, 1.0], dtype=float)
        self.vel.fill(0)
        self.rot.fill(0)
        self.omega.fill(0)
        self.motor_rpms.fill(12000.0)
        self.last_v_des.fill(0)

    def step(self, target_pos, target_yaw, dt, pids, v_ff=None, a_ff=None):
        pid_x, pid_y, pid_z = pids
        if dt <= 0:
            return

        # Safety guard against missing feedforward vectors
        if v_ff is None:
            v_ff = np.zeros(3, dtype=float)
        if a_ff is None:
            a_ff = np.zeros(3, dtype=float)

        # 1. Outer position loop -> Desired velocities with full feedforward (v_ff, a_ff)
        v_x_des = np.clip(pid_x.compute(target_pos[0], self.pos[0], dt, v_ff[0], a_ff[0]), -1.5, 1.5)
        v_y_des = np.clip(pid_y.compute(target_pos[1], self.pos[1], dt, v_ff[1], a_ff[1]), -1.5, 1.5)
        v_z_des = np.clip(pid_z.compute(target_pos[2], self.pos[2], dt, v_ff[2], a_ff[2]), -1.5, 1.5)
        self.last_v_des = np.array([v_x_des, v_y_des, v_z_des])

        # 2. Translation-orientation coupling (converting velocity error into desired angles)
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

        # 3. Smooth, stable angle tracking (inertial filter without jitter)
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

        # 4. Total thrust accounting for vertical acceleration a_ff[2]
        total_thrust = np.clip(self.mass * (self.g + a_ff[2] + err_vz * 4.0), 0.0, 2.0 * self.mass * self.g)

        # 5. Constant motor speed (no dynamic RPM mixing)
        self.motor_rpms.fill(12000.0)

        # 6. Translational dynamics with full axis coupling (Rotation Matrix)
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

        # Ground condition
        if self.pos[2] <= 0.0:
            self.pos[2] = 0.0
            self.vel.fill(0)
            self.omega.fill(0)
            self.rot.fill(0)

        self.prop_angle += 1500.0 * dt