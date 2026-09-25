import numpy as np

class TrajectoryGenerator:
    def __init__(self):
        self.mode = "MANUAL"  # MANUAL, CIRCLE, STEP, SPIRAL
        self.elapsed_time = 0.0
        self.active = False
        
        # Trajectory state vectors for feedforward use
        self.pos = np.zeros(3, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.acc = np.zeros(3, dtype=float)

    def start(self, mode="CIRCLE"):
        self.mode = mode
        self.elapsed_time = 0.0
        self.active = True

    def stop(self):
        self.mode = "MANUAL"
        self.active = False
        self.elapsed_time = 0.0
        self.pos = np.zeros(3, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.acc = np.zeros(3, dtype=float)

    def update(self, dt, manual_setpoint):
        if not self.active or self.mode == "MANUAL":
            self.pos = np.array(manual_setpoint, dtype=float)
            self.vel = np.zeros(3, dtype=float)
            self.acc = np.zeros(3, dtype=float)
            return self.pos

        self.elapsed_time += dt
        t = self.elapsed_time

        if self.mode == "CIRCLE":
            omega = 1.0 * np.pi / 10.0
            Rx, Ry = 0.9, 1.3
            
            # 1. Position
            x = Rx * np.cos(omega * t)
            y = Ry * np.sin(omega * t)
            z = 1.0
            self.pos = np.array([x, y, z], dtype=float)
            
            # 2. Velocity (first time derivative)
            vx = -Rx * omega * np.sin(omega * t)
            vy = Ry * omega * np.cos(omega * t)
            vz = 0.0
            self.vel = np.array([vx, vy, vz], dtype=float)
            
            # 3. Acceleration (second time derivative)
            ax = -Rx * (omega ** 2) * np.cos(omega * t)
            ay = -Ry * (omega ** 2) * np.sin(omega * t)
            az = 0.0
            self.acc = np.array([ax, ay, az], dtype=float)

        elif self.mode == "STEP":
            waypoints = [
                (0.0,  0.0,  1.0, 0.0),   
                (0.9,  0.0,  1.0, 3.0),   
                (0.9,  1.3,  1.1, 6.0),   
                (0.0,  1.3,  1.1, 9.0),   
                (0.0,  0.0,  1.2, 12.0),  
                (0.5,  0.6,  0.6, 15.0),  
                (0.0,  0.0,  1.0, 18.0)   
            ]
            
            active_wp = waypoints[-1]
            for i in range(len(waypoints) - 1):
                if waypoints[i][3] <= t < waypoints[i+1][3]:
                    active_wp = waypoints[i]
                    break
                    
            self.pos = np.array([active_wp[0], active_wp[1], active_wp[2]], dtype=float)
            # In step mode, velocity and acceleration at fixed waypoints are zero
            self.vel = np.zeros(3, dtype=float)
            self.acc = np.zeros(3, dtype=float)

        elif self.mode == "SPIRAL":
            omega = 0.2
            Rx, Ry = 0.9, 1.3
            
            # 1. Position
            x = Rx * np.cos(omega * t)
            y = Ry * np.sin(omega * t)
            z = 0.8 + 0.4 * np.sin(0.2 * t) 
            self.pos = np.array([x, y, z], dtype=float)
            
            # 2. Velocity
            vx = -Rx * omega * np.sin(omega * t)
            vy = Ry * omega * np.cos(omega * t)
            vz = 0.4 * 0.2 * np.cos(0.2 * t)
            self.vel = np.array([vx, vy, vz], dtype=float)
            
            # 3. Acceleration
            ax = -Rx * (omega ** 2) * np.cos(omega * t)
            ay = -Ry * (omega ** 2) * np.sin(omega * t)
            az = -0.4 * (0.2 ** 2) * np.sin(0.2 * t)
            self.acc = np.array([ax, ay, az], dtype=float)

        else:
            self.pos = np.array(manual_setpoint, dtype=float)
            self.vel = np.zeros(3, dtype=float)
            self.acc = np.zeros(3, dtype=float)

        return self.pos