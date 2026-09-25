import pygame
import numpy as np
import math

class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, initial_val, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = initial_val
        self.label = label
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
                self._update_val(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                self._update_val(event.pos[0])

    def _update_val(self, mouse_x):
        rel_x = max(0, min(mouse_x - self.rect.x, self.rect.w))
        self.val = self.min_val + (rel_x / self.rect.w) * (self.max_val - self.min_val)

    def draw(self, surface, font):
        text_surf = font.render(f"{self.label}: {self.val:.2f}", True, (210, 215, 225))
        surface.blit(text_surf, (self.rect.x, self.rect.y - 15))

        pygame.draw.rect(surface, (40, 45, 60), self.rect, border_radius=4)
        
        fill_w = int(self.rect.w * (self.val - self.min_val) / (self.max_val - self.min_val))
        fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_w, self.rect.h)
        pygame.draw.rect(surface, (70, 130, 240), fill_rect, border_radius=4)

class Button:
    def __init__(self, x, y, w, h, text):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.hovered = False
        self.active = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True
        return False

    def draw(self, surface, font):
        base_color = (40, 160, 90) if self.active else ((70, 140, 250) if self.hovered else (50, 60, 80))
        pygame.draw.rect(surface, base_color, self.rect, border_radius=5)
        text_surf = font.render(self.text, True, (255, 255, 255))
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)

class ControllerSelectorPanel:
    def __init__(self, x, y, total_w=476):
        self.x = x
        self.y = y
        self.current_algo = "PID"
        gap = 12
        w = (total_w - gap * 2) // 3
        h = 22
        
        self.btn_pid = Button(x, y, w, h, "PID")
        self.btn_adrc = Button(x + w + gap, y, w, h, "ADRC")
        self.btn_mpc = Button(x + (w + gap) * 2, y, w, h, "MPC")
        self.btn_pid.active = True

    def handle_event(self, event):
        if self.btn_pid.handle_event(event):
            self.current_algo = "PID"
            self.btn_pid.active = True
            self.btn_adrc.active = False
            self.btn_mpc.active = False
        elif self.btn_adrc.handle_event(event):
            self.current_algo = "ADRC"
            self.btn_pid.active = False
            self.btn_adrc.active = True
            self.btn_mpc.active = False
        elif self.btn_mpc.handle_event(event):
            self.current_algo = "MPC"
            self.btn_pid.active = False
            self.btn_adrc.active = False
            self.btn_mpc.active = True

    def draw(self, surface, font):
        lbl = font.render("Control Algorithm:", True, (180, 190, 205))
        surface.blit(lbl, (self.x, self.y - 16))
        self.btn_pid.draw(surface, font)
        self.btn_adrc.draw(surface, font)
        self.btn_mpc.draw(surface, font)

class TrajectoryPanel:
    def __init__(self, x, y, total_w=476):
        self.x = x
        self.y = y
        gap = 8
        w = (total_w - gap * 3) // 4
        h = 22
        
        self.btn_manual = Button(x, y, w, h, "Manual")
        self.btn_circle = Button(x + w + gap, y, w, h, "Circle")
        self.btn_step = Button(x + (w + gap) * 2, y, w, h, "Step")
        self.btn_spiral = Button(x + (w + gap) * 3, y, w, h, "Spiral")
        
        self.btn_manual.active = True

    def handle_event(self, event, traj_gen):
        if self.btn_manual.handle_event(event):
            traj_gen.stop()
            self.btn_manual.active = True
            self.btn_circle.active = False
            self.btn_step.active = False
            self.btn_spiral.active = False
        elif self.btn_circle.handle_event(event):
            traj_gen.start("CIRCLE")
            self.btn_manual.active = False
            self.btn_circle.active = True
            self.btn_step.active = False
            self.btn_spiral.active = False
        elif self.btn_step.handle_event(event):
            traj_gen.start("STEP")
            self.btn_manual.active = False
            self.btn_circle.active = False
            self.btn_step.active = True
            self.btn_spiral.active = False
        elif self.btn_spiral.handle_event(event):
            traj_gen.start("SPIRAL")
            self.btn_manual.active = False
            self.btn_circle.active = False
            self.btn_step.active = False
            self.btn_spiral.active = True

    def draw(self, surface, font):
        lbl = font.render("Trajectory Generator:", True, (180, 190, 205))
        surface.blit(lbl, (self.x, self.y - 16))
        self.btn_manual.draw(surface, font)
        self.btn_circle.draw(surface, font)
        self.btn_step.draw(surface, font)
        self.btn_spiral.draw(surface, font)

class SetpointPanel3D:
    def __init__(self, x, y, total_w=476):
        self.x = x
        self.y = y
        self.total_w = total_w
        self.rect = pygame.Rect(x, y - 12, total_w, 102)
        self.current_mode = "MOUSE" 
        
        btn_w = (total_w - 24) // 2
        btn_y = self.rect.y + 8
        self.btn_mouse = Button(x + 8, btn_y, btn_w, 22, "Mouse")
        self.btn_manual = Button(x + 8 + btn_w + 8, btn_y, btn_w, 22, "Send Manual")
        
        self.input_x = "0.0"
        self.input_y = "0.0"
        self.input_z = "1.0"
        self.active_field = None
        self.callback = None

    def set_callback(self, cb):
        self.callback = cb

    def update_text_fields(self, x, y, z):
        if self.active_field != "input_x": self.input_x = f"{x:.2f}"
        if self.active_field != "input_y": self.input_y = f"{y:.2f}"
        if self.active_field != "input_z": self.input_z = f"{z:.2f}"

    def handle_event(self, event):
        if self.btn_mouse.handle_event(event):
            self.current_mode = "MOUSE"
        if self.btn_manual.handle_event(event):
            self.current_mode = "MANUAL"
            self._send_values()

        if event.type == pygame.KEYDOWN:
            if self.active_field:
                curr_val = getattr(self, self.active_field)
                if event.key == pygame.K_BACKSPACE:
                    setattr(self, self.active_field, curr_val[:-1])
                elif event.key == pygame.K_RETURN:
                    self._send_values()
                    self.active_field = None
                else:
                    char = event.unicode
                    if char in "0123456789.-":
                        setattr(self, self.active_field, curr_val + char)

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            bx = self.x + 8
            by = self.rect.y + 52  
            field_w = (self.total_w - 32) // 3
            field_h = 24  
            step_x = field_w + 8                
            
            clicked_field = None
            if pygame.Rect(bx, by, field_w, field_h).collidepoint(event.pos):
                clicked_field = "input_x"
            elif pygame.Rect(bx + step_x, by, field_w, field_h).collidepoint(event.pos):
                clicked_field = "input_y"
            elif pygame.Rect(bx + step_x * 2, by, field_w, field_h).collidepoint(event.pos):
                clicked_field = "input_z"

            if clicked_field:
                if self.active_field != clicked_field:
                    setattr(self, clicked_field, "")
                self.active_field = clicked_field
                self.current_mode = "MANUAL"
            else:
                if not pygame.Rect(bx, by - 16, self.total_w - 16, field_h + 20).collidepoint(event.pos) and not self.rect_contains(event.pos):
                    if self.active_field and getattr(self, self.active_field) == "":
                        setattr(self, self.active_field, "0.0")
                    self.active_field = None

    def rect_contains(self, pos):
        return self.rect.collidepoint(pos)

    def _send_values(self):
        try:
            x = float(self.input_x) if self.input_x != "" else 0.0
            y = float(self.input_y) if self.input_y != "" else 0.0
            z = float(self.input_z) if self.input_z != "" else 1.0
            
            if self.callback:
                self.callback(x, y, z)
        except ValueError:
            pass

    def draw(self, surface, font):
        pygame.draw.rect(surface, (22, 27, 38), self.rect, border_radius=6)
        pygame.draw.rect(surface, (45, 55, 75), self.rect, 1, border_radius=6)

        self.btn_mouse.draw(surface, font)
        self.btn_manual.draw(surface, font)

        bx = self.x + 8
        label_y = self.rect.y + 35
        by = self.rect.y + 51
        field_w = (self.total_w - 32) // 3
        field_h = 24  
        step_x = field_w + 8
        labels = ["X:", "Y:", "Z:"]
        vals = [self.input_x, self.input_y, self.input_z]
        fields = ["input_x", "input_y", "input_z"]

        for i in range(3):
            fx = bx + (i * step_x)
            lbl = font.render(labels[i], True, (180, 190, 205))
            surface.blit(lbl, (fx, label_y))

            f_rect = pygame.Rect(fx, by, field_w, field_h)
            bg_col = (45, 55, 75) if self.active_field == fields[i] else (25, 30, 40)
            pygame.draw.rect(surface, bg_col, f_rect, border_radius=4)
            pygame.draw.rect(surface, (80, 95, 120), f_rect, 1, border_radius=4)

            val_surf = font.render(vals[i], True, (255, 255, 255))
            surface.blit(val_surf, (fx + 8, by + (field_h - val_surf.get_height()) // 2))

        mode_info = font.render(f"Mode: {self.current_mode}", True, (100, 220, 140))
        surface.blit(mode_info, (bx, self.rect.bottom - 18))
        
class DroneStatePanel:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)

    def draw(self, surface, font_bold, font_state, drone):
        pygame.draw.rect(surface, (22, 27, 38), self.rect, border_radius=6)
        pygame.draw.rect(surface, (45, 55, 75), self.rect, 1, border_radius=6)

        title_state = font_bold.render("DRONE STATE", True, (130, 210, 255))
        surface.blit(title_state, (self.rect.x + 8, self.rect.y + 5))

        px, py, pz = drone.pos[0], drone.pos[1], drone.pos[2]
        roll = math.degrees(drone.rot[0]) if hasattr(drone, 'rot') and len(drone.rot) > 0 else 0.0
        pitch = math.degrees(drone.rot[1]) if hasattr(drone, 'rot') and len(drone.rot) > 1 else 0.0
        
        vx = drone.velocity[0] if hasattr(drone, 'velocity') else (drone.last_v_des[0] if hasattr(drone, 'last_v_des') else 0.0)
        vy = drone.velocity[1] if hasattr(drone, 'velocity') else (drone.last_v_des[1] if hasattr(drone, 'last_v_des') else 0.0)
        vz = drone.velocity[2] if hasattr(drone, 'velocity') else (drone.last_v_des[2] if hasattr(drone, 'last_v_des') else 0.0)

        text_pos = font_state.render(f"Position: X={px:+.2f}  Y={py:+.2f}  Z={pz:+.2f}", True, (100, 220, 100))
        text_rot = font_state.render(f"Angles: Roll={roll:+.1f}°  Pitch={pitch:+.1f}°", True, (255, 180, 100))
        text_vel = font_state.render(f"Velocities: Vx={vx:+.2f}  Vy={vy:+.2f}  Vz={vz:+.2f}", True, (100, 200, 255))

        surface.blit(text_pos, (self.rect.x + 8, self.rect.y + 24))
        surface.blit(text_rot, (self.rect.x + 8, self.rect.y + 50))
        surface.blit(text_vel, (self.rect.x + 8, self.rect.y + 76))

class LoggerPanel:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
        self.btn_rect = pygame.Rect(self.rect.x + 8, self.rect.y + 38, w - 16, 30)

    def handle_event(self, event, logger, current_ticks, current_algo):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.btn_rect.collidepoint(event.pos):
                if not logger.is_recording:
                    logger.start_recording(current_ticks / 1000.0)
                else:
                    logger.stop_and_save(system_name=current_algo)
                return True
        return False

    def draw(self, surface, font_bold, font_small, logger):
        pygame.draw.rect(surface, (22, 27, 38), self.rect, border_radius=6)
        border_color = (220, 80, 80) if logger.is_recording else (45, 55, 75)
        pygame.draw.rect(surface, border_color, self.rect, 1, border_radius=6)

        logger_title = font_bold.render("CSV LOGGING", True, (130, 210, 255))
        surface.blit(logger_title, (self.rect.x + 8, self.rect.y + 4))

        status_str = f"Recording ({len(logger.recorded_data)})..." if logger.is_recording else "Ready to record"
        status_color = (100, 220, 100) if logger.is_recording else (180, 180, 180)
        text_logger_status = font_small.render(status_str, True, status_color)
        surface.blit(text_logger_status, (self.rect.x + 8, self.rect.y + 20))

        btn_bg = (180, 50, 50) if logger.is_recording else (40, 130, 80)
        pygame.draw.rect(surface, btn_bg, self.btn_rect, border_radius=4)
        btn_text_str = "STOP CSV" if logger.is_recording else "START CSV"
        btn_surface = font_bold.render(btn_text_str, True, (255, 255, 255))
        bx = self.btn_rect.x + (self.btn_rect.width - btn_surface.get_width()) // 2
        by = self.btn_rect.y + (self.btn_rect.height - btn_surface.get_height()) // 2
        surface.blit(btn_surface, (bx, by))

def draw_chart(surface, rect, font, title, min_val, max_val, lines_data):
    pygame.draw.rect(surface, (20, 24, 33), rect, border_radius=6)
    pygame.draw.rect(surface, (35, 42, 58), rect, 1, border_radius=6)

    title_surf = font.render(title, True, (160, 175, 200))
    surface.blit(title_surf, (rect.x + 8, rect.y + 3))

    plot_rect = pygame.Rect(rect.x + 45, rect.y + 20, rect.w - 53, rect.h - 24)
    pygame.draw.rect(surface, (12, 15, 20), plot_rect)

    max_lbl = font.render(f"{max_val:+.1f}", True, (130, 140, 160))
    mid_lbl = font.render(f"{(max_val+min_val)/2:+.1f}", True, (130, 140, 160))
    min_lbl = font.render(f"{min_val:+.1f}", True, (130, 140, 160))

    surface.blit(max_lbl, (rect.x + 4, plot_rect.y - 3))
    surface.blit(mid_lbl, (rect.x + 4, plot_rect.y + plot_rect.h // 2 - 6))
    surface.blit(min_lbl, (rect.x + 4, plot_rect.bottom - 9))

    for i in range(3):
        gy = plot_rect.y + int(plot_rect.h * i / 2)
        pygame.draw.line(surface, (25, 32, 45), (plot_rect.x, gy), (plot_rect.right, gy), 1)

    for item in lines_data:
        data = item["data"]
        color = item["color"]
        if len(data) < 2:
            continue
        
        points = []
        for idx, val in enumerate(data):
            px = plot_rect.x + int(idx * plot_rect.w / max(1, len(data) - 1))
            norm_v = (val - min_val) / (max_val - min_val if max_val != min_val else 1.0)
            py = plot_rect.bottom - int(norm_v * plot_rect.h)
            py = max(plot_rect.top, min(plot_rect.bottom, py))
            points.append((px, py))

        if len(points) >= 2:
            pygame.draw.aalines(surface, color, False, points)