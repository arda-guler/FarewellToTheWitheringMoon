import numpy as np
from scipy.spatial.transform import Rotation

from sound import *

class RigidBody:
    def __init__(self, model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia):
        self.model = model
        for idx_v, v in enumerate(self.model.vertices):
            self.model.vertices[idx_v] = v - CoM
        self.CoM = CoM
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.orient = orient
        self.ang_vel = ang_vel
        self.ang_accel = ang_accel
        self.mass = mass
        self.inertia = inertia

    # shifts center of mass
    def shift_CoM(self, shift):
        for idx_v, v in enumerate(self.model.vertices):
            self.model.vertices[idx_v] = v - shift

        self.CoM = self.CoM - shift

    def update_mass(self, mdot, dt):
        self.mass += mdot * dt

    def apply_torque(self, torque):
        inertia_inverse = np.linalg.inv(self.inertia)
        accel = np.dot(inertia_inverse, torque)
        self.ang_accel = self.ang_accel + accel

    def apply_force(self, force):
        accel = force / self.mass
        self.accel = self.accel + accel

    def apply_accel(self, accel):
        self.accel = self.accel + accel

    def rotate(self, dt):
        if np.linalg.norm(self.ang_vel) > 0:
            # Ensure the angular velocity is a column vector
            # angular_velocity = self.ang_vel.reshape(3, 1)
            axis = self.ang_vel / np.linalg.norm(self.ang_vel)
            angle_rad = np.linalg.norm(self.ang_vel) * dt

            rotation = Rotation.from_rotvec(angle_rad * axis)
    
            # Convert the rotation to a rotation matrix
            rotation_matrix = rotation.as_matrix()
    
            # Multiply the original orientation matrix by the rotation matrix
            self.orient = np.dot(rotation_matrix, self.orient)

            self.orient[0] = self.orient[0] / np.linalg.norm(self.orient[0])
            self.orient[1] = self.orient[1] / np.linalg.norm(self.orient[1])
            self.orient[2] = self.orient[2] / np.linalg.norm(self.orient[2])

    def clear_accels(self):
        self.accel = np.array([0, 0, 0])
        self.ang_accel = np.array([0, 0, 0])

    def update(self, dt):
        self.vel = self.vel + self.accel * dt
        self.pos = self.pos + self.vel * dt
        self.ang_vel = self.ang_vel + self.ang_accel * dt
        self.rotate(dt)
        self.clear_accels()

class Missile(RigidBody):
    def __init__(self, model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia,
                 max_thrust, throttle_range, throttle, prop_mass, mass_flow, target=None):
        super(Missile, self).__init__(model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia)
        self.max_thrust = max_thrust
        self.throttle_range = throttle_range
        self.throttle = throttle
        self.prop_mass = prop_mass
        self.mass_flow = mass_flow
        self.target = target

        self.thrust = self.throttle / 100 * self.max_thrust

    def drain_fuel(self, dt):
        self.update_mass(-self.mass_flow * self.throttle / 100, dt)

    def apply_thrust(self):
        if self.prop_mass <= 0:
            self.throttle = 0
            self.thrust = 0
            
        self.apply_force(self.orient[2] * self.thrust)

    def set_thrust_percent(self, percentage):
        if not percentage == 0:
            percentage = max(min(self.throttle_range[1], percentage), self.throttle_range[0])
        self.throttle = percentage
        self.thrust = self.max_thrust * percentage / 100

        if self.prop_mass <= 0:
            self.throttle = 0
            self.thrust = 0

class Ship(RigidBody):
    def __init__(self, model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia,
                 max_thrust, throttle_range, throttle, prop_mass, mass_flow, cy_h, cy_r):
        super(Ship, self).__init__(model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia)
        self.max_thrust = max_thrust
        self.throttle_range = throttle_range
        self.throttle = throttle
        self.prop_mass = prop_mass
        self.mass_flow = mass_flow
        self.cy_h = cy_h
        self.cy_r = cy_r

        self.thrust = self.throttle / 100 * self.max_thrust

    def drain_fuel(self, dt):
        self.update_mass(-self.mass_flow * self.throttle / 100, dt)

    def apply_thrust(self):
        if self.prop_mass <= 0:
            self.throttle = 0
            self.thrust = 0
            
        self.apply_force(self.orient[2] * self.thrust)

    def set_thrust_percent(self, percentage):
        if not percentage == 0:
            percentage = max(min(self.throttle_range[1], percentage), self.throttle_range[0])
        self.throttle = percentage
        self.thrust = self.max_thrust * percentage / 100

        if self.prop_mass <= 0:
            self.throttle = 0
            self.thrust = 0

    def shoot(self, weapon_index):
        self.weapons[weapon_index].shoot()

    def set_target(self, ntarget, tweapons=None):
        if not tweapons:
            for w in self.weapons:
                w.set_target(ntarget)

        else:
            for tw in tweapons:
                self.weapons[tw].set_target(ntarget)

class Bullet(RigidBody):
    def __init__(self, model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia,
                 size, timeout):
        super(Bullet, self).__init__(model, CoM, pos, vel, accel, orient, ang_vel, ang_accel, mass, inertia)
        self.size = size
        self.timeout = timeout

    def update_timer(self, objs, dt):
        self.timeout -= dt
        if self.timeout <= 0:
            objs.remove(self)
            del self

    def check_collision(self, objs):
        for o in objs:
            if not o == self:
                if np.linalg.norm(self.pos - o.pos) <= self.size:
                    return o
                
        return None
