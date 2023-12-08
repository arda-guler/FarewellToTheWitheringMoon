import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np
import random

from ui import *
from rigidbody import *

class Color:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

class Grid:
    def __init__(self, height=0, color=Color(0.1, 0.8, 0.1)):
        self.height = height
        self.color = color


def drawOrigin():
    glBegin(GL_LINES)
    glColor(1,0,0)
    glVertex3f(0,0,0)
    glVertex3f(100,0,0)
    glColor(0,1,0)
    glVertex3f(0,0,0)
    glVertex3f(0,100,0)
    glColor(0,0,1)
    glVertex3f(0,0,0)
    glVertex3f(0,0,100)
    glEnd()

def drawPoint(p, color):

    glColor(color.r, color.g, color.b)
        
    glPushMatrix()
    glTranslatef(p.pos[0], p.pos[1], p.pos[2])

    glBegin(GL_POINTS)
    glVertex3f(0, 0, 0)
    glEnd()

    glPopMatrix()

def drawGrid(cam, grid, objects, size=2000, divisions=50):
    spacing = 10**(int(math.log(abs(cam.pos[1] - grid.height) + 2, 10)) + 1)
    # size = abs(cam.pos.y) * 10

    # subdivisions
    scene_spacing = spacing / 10
    N = divisions * 10
    corner_x = (-cam.pos[0] - N * 0.5 * scene_spacing) + cam.pos[0] % (scene_spacing)
    corner_z = (-cam.pos[2] - N * 0.5 * scene_spacing) + cam.pos[2] % (scene_spacing)

    glColor(grid.color.r/2, grid.color.g/2, grid.color.b/2)
    glBegin(GL_LINES)
    for i in range(N + 1):
        cx = corner_x + i * scene_spacing
        z0 = corner_z
        z1 = corner_z + N * scene_spacing
        glVertex3f(cx, grid.height, z0)
        glVertex3f(cx, grid.height, z1)

    for i in range(N + 1):
        x0 = corner_x
        x1 = corner_x + N * scene_spacing
        cz = corner_z + i * scene_spacing
        glVertex3f(x0, grid.height, cz)
        glVertex3f(x1, grid.height, cz)
    glEnd()

    # superdivisions
    scene_spacing = spacing
    N = divisions
    corner_x = (-cam.pos[0] - N * 0.5 * scene_spacing) + cam.pos[0] % (scene_spacing)
    corner_z = (-cam.pos[2] - N * 0.5 * scene_spacing) + cam.pos[2] % (scene_spacing)

    glColor(grid.color.r, grid.color.g, grid.color.b)
    glBegin(GL_LINES)
    for i in range(N + 1):
        cx = corner_x + i * scene_spacing
        z0 = corner_z
        z1 = corner_z + N * scene_spacing
        glVertex3f(cx, grid.height, z0)
        glVertex3f(cx, grid.height, z1)

    for i in range(N + 1):
        x0 = corner_x
        x1 = corner_x + N * scene_spacing
        cz = corner_z + i * scene_spacing
        glVertex3f(x0, grid.height, cz)
        glVertex3f(x1, grid.height, cz)
    glEnd()

    # vertical lines
    glColor(grid.color.r, grid.color.g, grid.color.b)
    glBegin(GL_LINES)
    for o in objects:
        if isinstance(o, Ship):
            glVertex3f(o.pos[0], o.pos[1], o.pos[2])
            glVertex3f(o.pos[0], grid.height, o.pos[2])
    glEnd()

def drawForces(forces):
    
    for f in forces:
        glPushMatrix()

        scaler = 0.2
        start_position = f.point.pos
        end_position = f.point.pos + f.force
        f_vector = f.force * scaler
        
        f_dir = f_vector.normalized()
        arrowhead_start = f.force * scaler * 0.8

        if not f_dir.cross(vec3(1,0,0)) == vec3(0,0,0):
            arrowhead_vector1 = f_dir.cross(vec3(1,0,0))
        else:
            arrowhead_vector1 = f_dir.cross(vec3(0,1,0))

        arrowhead_vector2 = arrowhead_vector1.cross(f_dir)

        arrowhead_vector1 = arrowhead_vector1 * f.force.mag() * scaler * 0.1
        arrowhead_vector2 = arrowhead_vector2 * f.force.mag() * scaler * 0.1
            
        arrowhead_pt1 = arrowhead_start + arrowhead_vector1
        arrowhead_pt2 = arrowhead_start - arrowhead_vector1

        arrowhead_pt3 = arrowhead_start + arrowhead_vector2
        arrowhead_pt4 = arrowhead_start - arrowhead_vector2
        
        glTranslate(start_position.x, start_position.y, start_position.z)
        glColor(1,0,1)

        glBegin(GL_LINES)

        glVertex3f(0,0,0)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt1.x, arrowhead_pt1.y, arrowhead_pt1.z)
        glVertex3f(arrowhead_pt3.x, arrowhead_pt3.y, arrowhead_pt3.z)

        glVertex3f(arrowhead_pt2.x, arrowhead_pt2.y, arrowhead_pt2.z)
        glVertex3f(arrowhead_pt4.x, arrowhead_pt4.y, arrowhead_pt4.z)

        glVertex3f(arrowhead_pt2.x, arrowhead_pt2.y, arrowhead_pt2.z)
        glVertex3f(arrowhead_pt3.x, arrowhead_pt3.y, arrowhead_pt3.z)

        glVertex3f(arrowhead_pt1.x, arrowhead_pt1.y, arrowhead_pt1.z)
        glVertex3f(arrowhead_pt4.x, arrowhead_pt4.y, arrowhead_pt4.z)

        glVertex3f(arrowhead_pt1.x, arrowhead_pt1.y, arrowhead_pt1.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt2.x, arrowhead_pt2.y, arrowhead_pt2.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt3.x, arrowhead_pt3.y, arrowhead_pt3.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt4.x, arrowhead_pt4.y, arrowhead_pt4.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glEnd()

        glPopMatrix()

def generateStars(N=300):
    stars = []

    for n in range(N):
        phi = np.random.uniform(0, 2*np.pi)
        costheta = np.random.uniform(-1, 1)

        theta = np.arccos(costheta)

        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(theta)

        stars.append(np.array([x, y, z]) * 100e3)

    return stars

def drawStars(cam, stars):
    glColor(1, 1, 1)
    glPushMatrix()
    glTranslatef(cam.pos[0], cam.pos[1], cam.pos[2])
    glBegin(GL_POINTS)
    for s in stars:
        glVertex3f(s[0], s[1], s[2])
    glEnd()
    glPopMatrix()

def drawModel(model, pos, orient, scale, color=Color(1, 1, 1)):
    glPushMatrix()
    glTranslatef(pos[0], pos[1], pos[2])
    glColor(color.r, color.g, color.b)

    glBegin(GL_LINES)
    for lines in model.lines:
        v1 = model.vertices[lines[0]]
        v2 = model.vertices[lines[1]]

        v1_rot = np.array([[v1[0] * orient[0][0] + v1[1] * orient[1][0] + v1[2] * orient[2][0]],
                           [v1[0] * orient[0][1] + v1[1] * orient[1][1] + v1[2] * orient[2][1]],
                           [v1[0] * orient[0][2] + v1[1] * orient[1][2] + v1[2] * orient[2][2]]])

        v2_rot = np.array([[v2[0] * orient[0][0] + v2[1] * orient[1][0] + v2[2] * orient[2][0]],
                           [v2[0] * orient[0][1] + v2[1] * orient[1][1] + v2[2] * orient[2][1]],
                           [v2[0] * orient[0][2] + v2[1] * orient[1][2] + v2[2] * orient[2][2]]])
        
        glVertex3f(v1_rot[0], v1_rot[1], v1_rot[2])
        glVertex3f(v2_rot[0], v2_rot[1], v2_rot[2])
    glEnd()

    
    for faces in model.faces:
        glBegin(GL_POLYGON)
        vnum = len(faces)
        for i in range(vnum):
            v = model.vertices[faces[i]]
            v_rot = np.array([[v[0] * orient[0][0] + v[1] * orient[1][0] + v[2] * orient[2][0]],
                              [v[0] * orient[0][1] + v[1] * orient[1][1] + v[2] * orient[2][1]],
                              [v[0] * orient[0][2] + v[1] * orient[1][2] + v[2] * orient[2][2]]])

            glVertex3f(v_rot[0], v_rot[1], v_rot[2])
        glEnd()
    
    glPopMatrix()

def drawScene(cam, player, objects, stars):
    drawStars(cam, stars)
    drawGrid(cam, Grid(), objects)
    drawModel(player.model, player.pos, player.orient, 1, Color(1,1,1))

    for o in objects:
        if not o == player:
            if isinstance(o, Ship):
                drawModel(o.model, o.pos, o.orient, 1, Color(1, 0, 0))
            elif isinstance(o, Bullet):
                drawPoint(o, Color(1, 1, 1))

