import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

def draw_cylinder(radius, height, slices=16):
    quadric = gluNewQuadric()
    gluQuadricNormals(quadric, GLU_SMOOTH)
    gluCylinder(quadric, radius, radius, height, slices, 1)
    gluDeleteQuadric(quadric)

def apply_material(diffuse, specular=[0.3, 0.3, 0.3, 1.0], shininess=32.0):
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse)
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)

def draw_grid():
    glDisable(GL_LIGHTING)
    glLineWidth(1)
    glBegin(GL_LINES)
    glColor3f(0.18, 0.22, 0.28)
    for i in range(-40, 41):
        coord = i * 0.1
        glVertex3f(coord, -4.0, 0)
        glVertex3f(coord, 4.0, 0)
        glVertex3f(-4.0, coord, 0)
        glVertex3f(4.0, coord, 0)
    glEnd()
    glEnable(GL_LIGHTING)

def draw_bounding_box(min_x=-2.0, max_x=2.0, min_y=-2.0, max_y=2.0, min_z=0.0, max_z=2.5):
    glDisable(GL_LIGHTING)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
    glLineWidth(1.2)
    glColor4f(0.3, 0.6, 0.9, 0.35)
    
    # Base (z = min_z)
    glBegin(GL_LINE_LOOP)
    glVertex3f(min_x, min_y, min_z)
    glVertex3f(max_x, min_y, min_z)
    glVertex3f(max_x, max_y, min_z)
    glVertex3f(min_x, max_y, min_z)
    glEnd()

    # Upper frame (z = max_z)
    glBegin(GL_LINE_LOOP)
    glVertex3f(min_x, min_y, max_z)
    glVertex3f(max_x, min_y, max_z)
    glVertex3f(max_x, max_y, max_z)
    glVertex3f(min_x, max_y, max_z)
    glEnd()

    # Vertical posts
    glBegin(GL_LINES)
    glVertex3f(min_x, min_y, min_z); glVertex3f(min_x, min_y, max_z)
    glVertex3f(max_x, min_y, min_z); glVertex3f(max_x, min_y, max_z)
    glVertex3f(max_x, max_y, min_z); glVertex3f(max_x, max_y, max_z)
    glVertex3f(min_x, max_y, min_z); glVertex3f(min_x, max_y, max_z)
    glEnd()

    glDisable(GL_BLEND)
    glEnable(GL_LIGHTING)

def draw_drone_shadow(x, y, z):
    glDisable(GL_LIGHTING)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
    alpha = max(0.1, 0.6 - (z * 0.3))
    radius = 0.10 + (z * 0.03)
    
    glColor4f(0.0, 0.0, 0.0, alpha)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(x, y, 0.001)
    for angle in range(0, 361, 20):
        rad = math.radians(angle)
        glVertex3f(x + math.cos(rad) * radius, y + math.sin(rad) * radius, 0.001)
    glEnd()
    
    glDisable(GL_BLEND)
    glEnable(GL_LIGHTING)

def draw_drone_3d(drone):
    draw_drone_shadow(drone.pos[0], drone.pos[1], drone.pos[2])

    glPushMatrix()
    glTranslatef(drone.pos[0], drone.pos[1], drone.pos[2])
    
    glRotatef(math.degrees(drone.rot[2]), 0, 0, 1)
    glRotatef(math.degrees(drone.rot[1]), 0, 1, 0)
    glRotatef(math.degrees(drone.rot[0]), 1, 0, 0)

    # --- PCB BOARD ---
    apply_material([0.12, 0.14, 0.18, 1.0], specular=[0.4, 0.4, 0.4], shininess=32.0)
    glPushMatrix()
    glScalef(0.025, 0.025, 0.0025)
    for dx, dy, dz in [(0,0,1), (0,0,-1), (0,1,0), (0,-1,0), (1,0,0), (-1,0,0)]:
        glBegin(GL_QUADS)
        glNormal3f(float(dx), float(dy), float(dz))
        glVertex3f(-1, -1, dz); glVertex3f(1, -1, dz)
        glVertex3f(1, 1, dz); glVertex3f(-1, 1, dz)
        glEnd()
    glPopMatrix()

    # Central gold element (processor / flight controller)
    apply_material([0.85, 0.65, 0.12, 1.0], specular=[0.8, 0.8, 0.3], shininess=64.0)
    glPushMatrix()
    glTranslatef(0.0, 0.0, 0.003)
    glScalef(0.012, 0.016, 0.0006)
    glBegin(GL_QUADS)
    glNormal3f(0, 0, 1)
    glVertex3f(-1, -1, 1); glVertex3f(1, -1, 1)
    glVertex3f(1, 1, 1); glVertex3f(-1, 1, 1)
    glEnd()
    glPopMatrix()

    # --- NEW: DECORATIVE "X" MOTIF IN THE CENTER ---
    apply_material([0.25, 0.3, 0.4, 1.0], specular=[0.6, 0.6, 0.6], shininess=32.0)
    glLineWidth(3.0)
    glBegin(GL_LINES)
    # First X arm
    glVertex3f(-0.008, -0.008, 0.0036)
    glVertex3f(0.008, 0.008, 0.0036)
    # Second X arm
    glVertex3f(-0.008, 0.008, 0.0036)
    glVertex3f(0.008, -0.008, 0.0036)
    glEnd()

    # --- MINI-CAMERA / FRONT SENSOR ---
    apply_material([0.2, 0.2, 0.2, 1.0], specular=[0.5, 0.5, 0.5], shininess=32.0)
    glPushMatrix()
    glTranslatef(0.0, 0.020, 0.002)
    glScalef(0.004, 0.004, 0.003)
    for dx, dy, dz in [(0,0,1), (0,0,-1), (0,1,0), (0,-1,0), (1,0,0), (-1,0,0)]:
        glBegin(GL_QUADS)
        glNormal3f(float(dx), float(dy), float(dz))
        glVertex3f(-1, -1, dz); glVertex3f(1, -1, dz)
        glVertex3f(1, 1, dz); glVertex3f(-1, 1, dz)
        glEnd()
    glPopMatrix()

    # Battery underneath
    glPushMatrix()
    glTranslatef(0.0, 0.0, -0.007)
    glScalef(0.010, 0.020, 0.004)
    apply_material([0.6, 0.62, 0.65, 1.0])
    for dx, dy, dz in [(0,0,1), (0,0,-1), (0,1,0), (0,-1,0), (1,0,0), (-1,0,0)]:
        glBegin(GL_QUADS)
        glNormal3f(float(dx), float(dy), float(dz))
        glVertex3f(-1, -1, dz); glVertex3f(1, -1, dz)
        glVertex3f(1, 1, dz); glVertex3f(-1, 1, dz)
        glEnd()
    glPopMatrix()

    # Arms and motors
    arm_len = 0.075
    motors = [
        (arm_len * 0.7071, arm_len * 0.7071, True),
        (-arm_len * 0.7071, arm_len * 0.7071, True),
        (-arm_len * 0.7071, -arm_len * 0.7071, False),
        (arm_len * 0.7071, -arm_len * 0.7071, False)
    ]

    for mx, my, is_front in motors:
        glPushMatrix()
        apply_material([0.15, 0.15, 0.18, 1.0])
        glLineWidth(5)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(mx, my, 0)
        glEnd()
        glPopMatrix()

        glPushMatrix()
        glTranslatef(mx, my, -0.009)
        apply_material([0.8, 0.82, 0.85, 1.0], specular=[0.9, 0.9, 0.9], shininess=64.0)
        draw_cylinder(0.005, 0.018, slices=16)

        glTranslatef(0, 0, 0.019)
        glRotatef(drone.prop_angle if is_front else -drone.prop_angle, 0, 0, 1)
        
        # --- BRIGHT YELLOW BLADES (PROPELLERS) ---
        apply_material([1.0, 0.88, 0.15, 0.9], specular=[0.9, 0.9, 0.5], shininess=64.0)

        glBegin(GL_TRIANGLES)
        glNormal3f(0, 0, 1)
        glVertex3f(0, 0, 0.001)
        glVertex3f(0.032, 0.004, 0.0008)
        glVertex3f(0.032, -0.002, -0.0008)
        
        glVertex3f(0, 0, 0.001)
        glVertex3f(-0.032, -0.004, 0.0008)
        glVertex3f(-0.032, 0.002, -0.0008)
        glEnd()

        glPopMatrix()

    glPopMatrix()
    
def draw_target_3d(x, y, z):
    glDisable(GL_LIGHTING)
    glColor4f(0.2, 0.9, 0.4, 0.3)
    glLineWidth(1.5)
    glBegin(GL_LINES)
    glVertex3f(x, y, 0.0)
    glVertex3f(x, y, z)
    glEnd()

    glPointSize(8.0)
    glBegin(GL_POINTS)
    glColor3f(0.2, 0.95, 0.4)
    glVertex3f(x, y, z)
    glEnd()
    glEnable(GL_LIGHTING)

def render_scene(view3d_w, height, drone, setpoint_x, setpoint_y, setpoint_z):
    glClearColor(0.08, 0.09, 0.12, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    glLightfv(GL_LIGHT0, GL_POSITION, [2.0, -4.0, 5.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.9, 0.95, 1.0, 1.0])

    draw_grid()
    draw_bounding_box(-2.0, 2.0, -2.0, 2.0, 0.0, 2.5)
    draw_target_3d(setpoint_x, setpoint_y, setpoint_z)
    draw_drone_3d(drone)

def render_gui_panel(gui_surface, gui_texture, panel_w, height):
    gui_data = pygame.image.tostring(gui_surface, "RGB", True)

    glViewport(panel_w[0], 0, panel_w[1], height)
    glMatrixMode(GL_PROJECTION); glLoadIdentity()
    glOrtho(0, panel_w[1], 0, height, -1, 1)
    glMatrixMode(GL_MODELVIEW); glLoadIdentity()
    
    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)

    glEnable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, gui_texture)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, panel_w[1], height, 0, GL_RGB, GL_UNSIGNED_BYTE, gui_data)

    glColor3f(1.0, 1.0, 1.0)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 0); glVertex2f(0, 0)
    glTexCoord2f(1, 0); glVertex2f(panel_w[1], 0)
    glTexCoord2f(1, 1); glVertex2f(panel_w[1], height)
    glTexCoord2f(0, 1); glVertex2f(0, height)
    glEnd()
    glDisable(GL_TEXTURE_2D)