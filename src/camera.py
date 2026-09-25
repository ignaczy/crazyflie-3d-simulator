import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *

def setup_camera(view_width, height):
    glViewport(0, 0, view_width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (view_width / height), 0.01, 50.0)
    
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    # Fixed camera position configured around the center of the scene (0, 0, 0.5)
    gluLookAt(0.0, -2.5, 1.8, 
              0.0, 0.0, 0.5, 
              0.0, 0.0, 1.0)

def get_3d_point_from_mouse(mouse_x, mouse_y, viewport_h, target_z):
    modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
    projection = glGetDoublev(GL_PROJECTION_MATRIX)
    viewport = glGetIntegerv(GL_VIEWPORT)

    win_x = float(mouse_x)
    win_y = float(viewport_h - mouse_y)

    near_point = gluUnProject(win_x, win_y, 0.0, modelview, projection, viewport)
    far_point = gluUnProject(win_x, win_y, 1.0, modelview, projection, viewport)

    if near_point is None or far_point is None:
        return 0.0, 0.0

    ray_dir = np.array(far_point) - np.array(near_point)
    if abs(ray_dir[2]) < 1e-6:
        return 0.0, 0.0

    t = (target_z - near_point[2]) / ray_dir[2]
    intersection = np.array(near_point) + t * ray_dir

    return float(intersection[0]), float(intersection[1])