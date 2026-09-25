import sys
import math
import traceback
import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *

from config import WIDTH, HEIGHT, VIEW3D_W, PANEL_W
from controllers import PIDController, ADRCController, MPCController
from trajectory import TrajectoryGenerator
from drone import Crazyflie3D
from ui import Slider, Button, SetpointPanel3D, ControllerSelectorPanel, TrajectoryPanel, DroneStatePanel, LoggerPanel, draw_chart
from camera import setup_camera, get_3d_point_from_mouse
from renderer import render_scene, render_gui_panel
from logger import DataLogger

def render_logger_overlay(surface, texture_id, x, y, w, h, screen_w, screen_h, font_bold, font_small, logger_panel, logger_obj):
    surface.fill((22, 27, 38))
    logger_panel.draw(surface, font_bold, font_small, logger_obj)

    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, screen_w, screen_h, 0, -1, 1)

    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    glDisable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glEnable(GL_TEXTURE_2D)

    data = pygame.image.tostring(surface, "RGB", True)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data)

    glColor4f(1.0, 1.0, 1.0, 1.0)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 0); glVertex2f(x, y + h)
    glTexCoord2f(1, 0); glVertex2f(x + w, y + h)
    glTexCoord2f(1, 1); glVertex2f(x + w, y)
    glTexCoord2f(0, 1); glVertex2f(x, y)
    glEnd()

    glDisable(GL_TEXTURE_2D)
    glEnable(GL_DEPTH_TEST)

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

class SimulatorApp:
    def __init__(self):
        pygame.init()
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Crazyflie 2.1+ | Advanced Cascade Control Simulator")

        self.gui_surface = pygame.Surface((PANEL_W, HEIGHT))
        self.logger_surface = pygame.Surface((222, 75))

        self.font_small = pygame.font.SysFont("Arial", 13)
        self.font_bold = pygame.font.SysFont("Arial", 14, bold=True)
        self.font_state = pygame.font.SysFont("Arial", 15)

        self.gui_texture = glGenTextures(1)
        self.logger_texture = glGenTextures(1)

        self.drone = Crazyflie3D()
        self.traj_gen = TrajectoryGenerator()
        self.logger = DataLogger()

        self.current_algo = "PID"
        self.pids = self.create_controllers(self.current_algo)
        self.pid_x, self.pid_y, self.pid_z = self.pids

        panel_x = 12
        slider_w = 476

        self.algo_panel = ControllerSelectorPanel(panel_x, 12, slider_w)
        self.traj_panel = TrajectoryPanel(panel_x, 58, slider_w)

        self.slider_1 = Slider(panel_x, 102, slider_w, 13, 0.0, 5.0, self.pid_x.Kp, "Kp (X/Y)")
        self.slider_2 = Slider(panel_x, 132, slider_w, 13, 0.0, 3.0, self.pid_x.Kd, "Kd (X/Y)")
        self.slider_3 = Slider(panel_x, 162, slider_w, 13, 0.0, 8.0, self.pid_z.Kp, "Kp (Z)")
        self.slider_4 = Slider(panel_x, 192, slider_w, 13, 0.0, 3.0, self.pid_z.Kd, "Kd (Z)")
        self.slider_5 = Slider(panel_x, 222, slider_w, 13, 1.0, 40.0, 15.0, "wc (Z)")
        self.slider_6 = Slider(panel_x, 252, slider_w, 13, 10.0, 100.0, 30.0, "wo (Z)")

        self.btn_reset = Button(panel_x, 285, slider_w, 24, "STATE RESET (SPACE)")

        self.chart_w = slider_w
        self.chart_h = 76
        self.chart_y_starts = [320, 400, 480, 560, 640]

        self.setpoint_panel = SetpointPanel3D(x=panel_x, y=735, total_w=slider_w)
        self.state_panel = DroneStatePanel(x=panel_x, y=845, w=slider_w, h=110)

        self.logger_panel = LoggerPanel(0, 0, 222, 75)
        self.logger_screen_rect = pygame.Rect(20, HEIGHT - 95, 222, 75)

        self.setpoint_x, self.setpoint_y, self.setpoint_z = 0.0, 0.0, 1.0

        def apply_setpoint(x, y, z):
            self.setpoint_x, self.setpoint_y, self.setpoint_z = x, y, z

        self.setpoint_panel.set_callback(apply_setpoint)

        self.hist_sp_x, self.hist_pv_x = [], []
        self.hist_sp_y, self.hist_pv_y = [], []
        self.hist_sp_z, self.hist_pv_z = [], []
        self.hist_pv_roll, self.hist_pv_pitch = [], []
        self.hist_vx, self.hist_vy, self.hist_vz = [], [], []
        self.MAX_HIST = 200

        self.running = True

    def create_controllers(self, algo_type):
        if algo_type == "ADRC":
            return (
                ADRCController(b0=3.0, wc=9.0, wo=40.0, limit=1.0),
                ADRCController(b0=3.0, wc=9.0, wo=40.0, limit=1.0),
                ADRCController(b0=5.0,  wc=8.0, wo=30.0, limit=1.0)
            )
        elif algo_type == "MPC":
            return (
                MPCController(horizon=20, Q_pos=25.0, R_u=0.1, limit=1.0),
                MPCController(horizon=20, Q_pos=25.0, R_u=0.1, limit=1.0),
                MPCController(horizon=20, Q_pos=25.0, R_u=0.1, limit=1.0)
            )
        else:
            return (
                PIDController(Kp=2.0, Ki=0.0, Kd=0.5, limit=1.0),
                PIDController(Kp=2.0, Ki=0.0, Kd=0.5, limit=1.0),
                PIDController(Kp=5.0, Ki=0.0, Kd=0.0, limit=1.0)
            )

    def trigger_reset(self):
        self.setpoint_x, self.setpoint_y, self.setpoint_z = 0.0, 0.0, 1.0
        self.setpoint_panel.update_text_fields(0.0, 0.0, 1.0)
        self.traj_panel.btn_manual.active = True
        self.traj_panel.btn_circle.active = False
        self.traj_panel.btn_step.active = False
        self.traj_panel.btn_spiral.active = False
        self.traj_gen.stop()
        self.drone.reset_state()
        for p in self.pids:
            p.reset()

    def run(self):
        clock = pygame.time.Clock()
        last_ticks = pygame.time.get_ticks()

        while self.running:
            current_ticks = pygame.time.get_ticks()
            dt = (current_ticks - last_ticks) / 1000.0
            last_ticks = current_ticks
            if dt > 0.05: dt = 0.016

            self.handle_algorithm_switching()
            self.handle_events(current_ticks)
            self.update_controllers_parameters()
            self.update_simulation(dt, current_ticks)
            self.render(current_ticks)

            clock.tick(60)

        pygame.quit()

    def handle_algorithm_switching(self):
        if self.algo_panel.current_algo != self.current_algo:
            self.current_algo = self.algo_panel.current_algo
            self.pids = self.create_controllers(self.current_algo)
            self.pid_x, self.pid_y, self.pid_z = self.pids

            if self.current_algo == "ADRC":
                self.slider_1.label, self.slider_1.min_val, self.slider_1.max_val, self.slider_1.val = "b0 (X/Y)", 1.0, 30.0, self.pid_x.b0
                self.slider_2.label, self.slider_2.min_val, self.slider_2.max_val, self.slider_2.val = "wc (X/Y)", 1.0, 40.0, self.pid_x.wc
                self.slider_3.label, self.slider_3.min_val, self.slider_3.max_val, self.slider_3.val = "wo (X/Y)", 10.0, 100.0, self.pid_x.wo
                self.slider_4.label, self.slider_4.min_val, self.slider_4.max_val, self.slider_4.val = "b0 (Z)", 1.0, 30.0, self.pid_z.b0
                self.slider_5.label, self.slider_5.min_val, self.slider_5.max_val, self.slider_5.val = "wc (Z)", 1.0, 40.0, self.pid_z.wc
                self.slider_6.label, self.slider_6.min_val, self.slider_6.max_val, self.slider_6.val = "wo (Z)", 10.0, 100.0, self.pid_z.wo
            elif self.current_algo == "MPC":
                self.slider_1.label, self.slider_1.min_val, self.slider_1.max_val, self.slider_1.val = "Horizon N (X/Y)", 2.0, 30.0, float(self.pid_x.N)
                self.slider_2.label, self.slider_2.min_val, self.slider_2.max_val, self.slider_2.val = "Q Weight (X/Y)", 1.0, 50.0, self.pid_x.Q
                self.slider_3.label, self.slider_3.min_val, self.slider_3.max_val, self.slider_3.val = "R Weight (X/Y)", 0.1, 10.0, self.pid_x.R
                self.slider_4.label, self.slider_4.min_val, self.slider_4.max_val, self.slider_4.val = "Horizon N (Z)", 2.0, 30.0, float(self.pid_z.N)
                self.slider_5.label, self.slider_5.min_val, self.slider_5.max_val, self.slider_5.val = "Q Weight (Z)", 1.0, 50.0, self.pid_z.Q
                self.slider_6.label, self.slider_6.min_val, self.slider_6.max_val, self.slider_6.val = "R Weight (Z)", 0.1, 10.0, self.pid_z.R
            else:
                self.slider_1.label, self.slider_1.min_val, self.slider_1.max_val, self.slider_1.val = "Kp (X/Y)", 0.0, 5.0, self.pid_x.Kp
                self.slider_2.label, self.slider_2.min_val, self.slider_2.max_val, self.slider_2.val = "Kd (X/Y)", 0.0, 3.0, self.pid_x.Kd
                self.slider_3.label, self.slider_3.min_val, self.slider_3.max_val, self.slider_3.val = "Kp (Z)", 0.0, 8.0, self.pid_z.Kp
                self.slider_4.label, self.slider_4.min_val, self.slider_4.max_val, self.slider_4.val = "Kd (Z)", 0.0, 3.0, self.pid_z.Kd
                self.slider_5.label = ""
                self.slider_6.label = ""

    def handle_events(self, current_ticks):
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False

            if event.type == KEYDOWN and event.key == K_SPACE:
                self.trigger_reset()

            if event.type in (MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION):
                if event.pos[0] < VIEW3D_W:
                    if event.type == MOUSEBUTTONDOWN and event.button == 1:
                        if self.logger_screen_rect.collidepoint(event.pos):
                            local_pos = (event.pos[0] - self.logger_screen_rect.x, event.pos[1] - self.logger_screen_rect.y)
                            local_event = pygame.event.Event(event.type, {'pos': local_pos, 'button': event.button})
                            self.logger_panel.handle_event(local_event, self.logger, current_ticks, self.current_algo)
                        elif self.setpoint_panel.current_mode == "MOUSE":
                            mx, my = event.pos
                            setup_camera(VIEW3D_W, HEIGHT)
                            wx, wy = get_3d_point_from_mouse(mx, my, HEIGHT, self.setpoint_z)
                            
                            self.setpoint_x = float(np.clip(wx, -2.0, 2.0))
                            self.setpoint_y = float(np.clip(wy, -2.0, 2.0))
                            self.setpoint_panel.update_text_fields(self.setpoint_x, self.setpoint_y, self.setpoint_z)

                elif event.pos[0] >= VIEW3D_W:
                    local_pos = (event.pos[0] - VIEW3D_W, event.pos[1])
                    event_panel = pygame.event.Event(event.type, {'pos': local_pos, **{k: v for k, v in event.__dict__.items() if k not in ['pos']}})
                    
                    self.slider_1.handle_event(event_panel)
                    self.slider_2.handle_event(event_panel)
                    self.slider_3.handle_event(event_panel)
                    self.slider_4.handle_event(event_panel)
                    
                    if self.current_algo in ("ADRC", "MPC"):
                        self.slider_5.handle_event(event_panel)
                        self.slider_6.handle_event(event_panel)

                    if event.type == MOUSEBUTTONDOWN and event.button == 1:
                        self.algo_panel.handle_event(event_panel)
                        self.traj_panel.handle_event(event_panel, self.traj_gen)
                        if self.btn_reset.handle_event(event_panel):
                            self.trigger_reset()

                    self.setpoint_panel.handle_event(event_panel)
            else:
                if hasattr(event, 'pos') and event.pos[0] >= VIEW3D_W:
                    local_pos = (event.pos[0] - VIEW3D_W, event.pos[1])
                    event_panel = pygame.event.Event(event.type, event.__dict__)
                    event_panel.pos = local_pos
                    self.setpoint_panel.handle_event(event_panel)
                else:
                    self.setpoint_panel.handle_event(event)

    def update_controllers_parameters(self):
        if self.current_algo == "PID":
            self.pid_x.Kp = self.pid_y.Kp = self.slider_1.val
            self.pid_x.Kd = self.pid_y.Kd = self.slider_2.val
            self.pid_z.Kp = self.slider_3.val
            self.pid_z.Kd = self.slider_4.val
        elif self.current_algo == "ADRC":
            self.pid_x.b0 = self.pid_y.b0 = self.slider_1.val
            self.pid_x.wc = self.pid_y.wc = self.slider_2.val
            self.pid_x.wo = self.pid_y.wo = self.slider_3.val
            
            self.pid_z.b0 = self.slider_4.val
            self.pid_z.wc = self.slider_5.val
            self.pid_z.wo = self.slider_6.val
        else: # MPC
            self.pid_x.N = self.pid_y.N = int(self.slider_1.val)
            self.pid_x.Q = self.pid_y.Q = self.slider_2.val
            self.pid_x.R = self.pid_y.R = self.slider_3.val
            
            self.pid_z.N = int(self.slider_4.val)
            self.pid_z.Q = self.slider_5.val
            self.pid_z.R = self.slider_6.val

    def update_simulation(self, dt, current_ticks):
        target_pos = np.array([self.setpoint_x, self.setpoint_y, self.setpoint_z])

        if dt > 0:
            manual_target = np.array([self.setpoint_x, self.setpoint_y, self.setpoint_z])
            target_pos = self.traj_gen.update(dt, manual_target)
            
            v_ff = getattr(self.traj_gen, 'vel', np.zeros(3))
            a_ff = getattr(self.traj_gen, 'acc', getattr(self.traj_gen, 'accel', np.zeros(3)))
            
            self.drone.step(target_pos, 0.0, dt, self.pids, v_ff=v_ff, a_ff=a_ff)

            self.hist_sp_x.append(target_pos[0]); self.hist_pv_x.append(self.drone.pos[0])
            self.hist_sp_y.append(target_pos[1]); self.hist_pv_y.append(self.drone.pos[1])
            self.hist_sp_z.append(target_pos[2]); self.hist_pv_z.append(self.drone.pos[2])
            
            self.hist_pv_roll.append(math.degrees(self.drone.rot[0]))
            self.hist_pv_pitch.append(math.degrees(self.drone.rot[1]))

            self.hist_vx.append(self.drone.last_v_des[0])
            self.hist_vy.append(self.drone.last_v_des[1])
            self.hist_vz.append(self.drone.last_v_des[2])

            if len(self.hist_sp_x) > self.MAX_HIST:
                self.hist_sp_x.pop(0); self.hist_pv_x.pop(0)
                self.hist_sp_y.pop(0); self.hist_pv_y.pop(0)
                self.hist_sp_z.pop(0); self.hist_pv_z.pop(0)
                self.hist_pv_roll.pop(0); self.hist_pv_pitch.pop(0)
                self.hist_vx.pop(0); self.hist_vy.pop(0); self.hist_vz.pop(0)

        self._last_target_pos = target_pos
        
        class SystemWrapper:
            def __init__(self, app):
                self.app = app
            def get_charts_data(self):
                return {
                    "X_pos": [self.app.hist_sp_x, self.app.hist_pv_x],
                    "Y_pos": [self.app.hist_sp_y, self.app.hist_pv_y],
                    "Z_pos": [self.app.hist_sp_z, self.app.hist_pv_z],
                    "Roll_Pitch": [self.app.hist_pv_roll, self.app.hist_pv_pitch],
                    "Velocities": [self.app.hist_vx, self.app.hist_vy, self.app.hist_vz]
                }
                
        if dt > 0:
            # Calculating errors (target - current) for X, Y, Z axes
            errs = {
                "x": target_pos[0] - self.drone.pos[0],
                "y": target_pos[1] - self.drone.pos[1],
                "z": target_pos[2] - self.drone.pos[2]
            }
            # Passing errors to the logger
            self.logger.sample(current_ticks / 1000.0, SystemWrapper(self), errors=errs)

    def render(self, current_ticks):
        target_pos = getattr(self, '_last_target_pos', np.array([self.setpoint_x, self.setpoint_y, self.setpoint_z]))

        setup_camera(VIEW3D_W, HEIGHT)
        render_scene(VIEW3D_W, HEIGHT, self.drone, target_pos[0], target_pos[1], target_pos[2])

        render_logger_overlay(self.logger_surface, self.logger_texture, 
                              self.logger_screen_rect.x, self.logger_screen_rect.y, 
                              self.logger_screen_rect.width, self.logger_screen_rect.height, 
                              VIEW3D_W, HEIGHT, self.font_bold, self.font_small, self.logger_panel, self.logger)

        self.gui_surface.fill((15, 17, 24))
        
        self.algo_panel.draw(self.gui_surface, self.font_small)
        self.traj_panel.draw(self.gui_surface, self.font_small)
        
        self.slider_1.draw(self.gui_surface, self.font_small)
        self.slider_2.draw(self.gui_surface, self.font_small)
        self.slider_3.draw(self.gui_surface, self.font_small)
        self.slider_4.draw(self.gui_surface, self.font_small)
        
        if self.current_algo in ("ADRC", "MPC"):
            self.slider_5.draw(self.gui_surface, self.font_small)
            self.slider_6.draw(self.gui_surface, self.font_small)

        self.btn_reset.draw(self.gui_surface, self.font_small)

        panel_x = 12
        draw_chart(self.gui_surface, pygame.Rect(panel_x, self.chart_y_starts[0], self.chart_w, self.chart_h), self.font_small,
                    "X Position [m]", -2.0, 2.0,
                    [{"data": self.hist_sp_x, "color": (80, 220, 120)}, {"data": self.hist_pv_x, "color": (100, 200, 255)}])

        draw_chart(self.gui_surface, pygame.Rect(panel_x, self.chart_y_starts[1], self.chart_w, self.chart_h), self.font_small,
                    "Y Position [m]", -2.0, 2.0,
                    [{"data": self.hist_sp_y, "color": (80, 220, 120)}, {"data": self.hist_pv_y, "color": (200, 100, 255)}])

        draw_chart(self.gui_surface, pygame.Rect(panel_x, self.chart_y_starts[2], self.chart_w, self.chart_h), self.font_small,
                    "Z Altitude [m]", 0.0, 2.5,
                    [{"data": self.hist_sp_z, "color": (80, 220, 120)}, {"data": self.hist_pv_z, "color": (255, 180, 80)}])

        draw_chart(self.gui_surface, pygame.Rect(panel_x, self.chart_y_starts[3], self.chart_w, self.chart_h), self.font_small,
                    "Roll / Pitch Angles [°]", -15.0, 15.0,
                    [
                        {"data": self.hist_pv_roll, "color": (255, 100, 100)},
                        {"data": self.hist_pv_pitch, "color": (255, 255, 100)}
                    ])

        draw_chart(self.gui_surface, pygame.Rect(panel_x, self.chart_y_starts[4], self.chart_w, self.chart_h), self.font_small,
                    "Control Signals Vx, Vy, Vz [m/s]", -1.2, 1.2,
                    [
                        {"data": self.hist_vx, "color": (100, 200, 255)},
                        {"data": self.hist_vy, "color": (200, 100, 255)},
                        {"data": self.hist_vz, "color": (255, 180, 80)}
                    ])

        self.setpoint_panel.draw(self.gui_surface, self.font_small)
        self.state_panel.draw(self.gui_surface, self.font_bold, self.font_state, self.drone)

        render_gui_panel(self.gui_surface, self.gui_texture, (VIEW3D_W, PANEL_W), HEIGHT)

        pygame.display.flip()