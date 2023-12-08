import numpy as np
import random

from rigidbody import *
from model import *
from sound import *

class Weapon:
    def __init__(self, parent, rel_pos):
        self.parent = parent
        self.rel_pos = rel_pos

class Machinegun(Weapon):
    def __init__(self, parent, rel_pos, muzzle_vel, reload_time,
                 num_projectiles, sound_channel):
        self.parent = parent
        self.rel_pos = rel_pos
        self.muzzle_vel = muzzle_vel
        self.reload_time = reload_time
        self.reload_countdown = 0
        self.num_projectiles = num_projectiles
        self.target = None
        self.sounds = ["machinegun1",
                       "machinegun2",
                       "machinegun3",
                       "machinegun4"]

        self.sound_channel = sound_channel

    def antiselfshoot(self, shoot_dir):
        p0 = self.get_abs_pos()
        v = shoot_dir
        cylinder_center = self.parent.pos
        cylinder_axis = self.parent.orient[2]
        cylinder_radius = self.parent.cy_r
        cylinder_height = self.parent.cy_h

        # Parametric equation of the line
        def line(t):
            return p0 + t * v

        # Check intersection with cylinder
        a = cylinder_axis
        b = p0 - cylinder_center - np.dot(p0 - cylinder_center, a) * a
        c = cylinder_radius
        h = cylinder_height

        # Coefficients of the quadratic equation
        A = np.dot(v - np.dot(v, a) * a, v - np.dot(v, a) * a)
        B = 2 * np.dot(v - np.dot(v, a) * a, b - np.dot(b, a) * a)
        C = np.dot(b - np.dot(b, a) * a, b - np.dot(b, a) * a) - c**2

        # Discriminant of the quadratic equation
        discriminant = B**2 - 4 * A * C

        if discriminant < 0:
            # No intersection
            return False
        else:
            # Find the parameter t for the intersection point
            t1 = (-B + np.sqrt(discriminant)) / (2 * A)
            t2 = (-B - np.sqrt(discriminant)) / (2 * A)

            # Check if the intersection point is within the height of the cylinder
            z1 = np.dot(line(t1) - cylinder_center, a)
            z2 = np.dot(line(t2) - cylinder_center, a)

            if 0 <= z1 <= h or 0 <= z2 <= h:
                return True
            else:
                return False

    def is_shooting_self(self, shoot_dir):
        point = self.get_abs_pos() + shoot_dir * 5
        
        # Vector from the cylinder center to the point
        vector_to_point = point - self.parent.pos

        # Project the vector onto the cylinder axis
        projection_on_axis = np.dot(vector_to_point, self.parent.orient[2])
        
        # Calculate the distance between the point and the axis
        distance_to_axis = np.linalg.norm(vector_to_point - projection_on_axis * self.parent.orient[2])

        # Check if the point is within the radius and height of the cylinder
        is_inside_radius = distance_to_axis <= self.parent.cy_r
        is_inside_height = -self.parent.cy_h <= projection_on_axis <= self.parent.cy_h

        return is_inside_radius and is_inside_height

    def get_abs_pos(self):
        dp = np.array([self.parent.orient[0][0] * self.rel_pos[0] + self.parent.orient[1][0] * self.rel_pos[1] + self.parent.orient[2][0] * self.rel_pos[2],
                       self.parent.orient[0][1] * self.rel_pos[0] + self.parent.orient[1][1] * self.rel_pos[1] + self.parent.orient[2][1] * self.rel_pos[2],
                       self.parent.orient[0][2] * self.rel_pos[0] + self.parent.orient[1][2] * self.rel_pos[1] + self.parent.orient[2][2] * self.rel_pos[2]])
        return self.parent.pos + dp #np.dot(self.parent.orient, self.rel_pos)

    def get_shoot_dir(self):
        if not self.target:
            direction = self.get_abs_pos() - self.parent.pos
            return direction / np.linalg.norm(direction)

        self_abs_pos = self.get_abs_pos()
        target_dir = self.target.pos - self_abs_pos
        dist = np.linalg.norm(target_dir)
        
        projectile_time = dist / self.muzzle_vel
        target_travel = (self.target.vel - self.parent.vel) * projectile_time
        shoot_pos = self.target.pos + target_travel

        shoot_dir = shoot_pos - self_abs_pos
        return shoot_dir / np.linalg.norm(shoot_dir)

    def create_bullet_orientation(self, direction):
        if direction[0] != 0 or direction[1] != 0:
            perpendicular = np.array([-direction[1], direction[0], 0])
        else:
            perpendicular = np.array([1, 0, 0])

        perpendicular /= np.linalg.norm(perpendicular)
        up = np.cross(direction, perpendicular)

        matrix = np.column_stack((perpendicular, up, direction))

        return matrix

    def shoot(self, objects):
        if self.reload_countdown > 0 or self.num_projectiles <= 0:
            return

        shoot_dir = self.get_shoot_dir()
        if self.is_shooting_self(shoot_dir) == True:
            return

        self.num_projectiles -= 1
        self.reload_countdown = self.reload_time

        bullet_size = 1
        bullet_model = Model("bullet")
        bullet_CoM = np.array([0, 0, 0])
        bullet_pos = self.get_abs_pos() + shoot_dir * bullet_size * 1.5
        bullet_vel = self.parent.vel + shoot_dir * self.muzzle_vel
        bullet_accel = np.array([0,0,0])
        bullet_orient = self.create_bullet_orientation(shoot_dir)
        bullet_ang_vel = np.array([0,0,0])
        bullet_ang_accel = np.array([0,0,0])
        bullet_mass = 0.05
        bullet_inertia = np.array([[50, 0, 0],
                                   [0, 50, 0],
                                   [0, 0, 50]])

        target_dir = self.target.pos - self.get_abs_pos()
        dist = np.linalg.norm(target_dir)
        
        projectile_time = dist / self.muzzle_vel
        
        new_projectile = Bullet(bullet_model, bullet_CoM, bullet_pos, bullet_vel, bullet_accel,
                                bullet_orient, bullet_ang_vel, bullet_ang_accel,
                                bullet_mass, bullet_inertia, bullet_size, projectile_time * 1.5)

        objects.append(new_projectile)

        play_sfx(random.choice(self.sounds), 0, self.sound_channel, 0.3)

    def update(self, dt):
        if self.reload_countdown > 0:
            self.reload_countdown -= dt

        if self.reload_countdown < 0:
            self.reload_countdown = 0

    def set_target(self, ntarget):
        self.target = ntarget
