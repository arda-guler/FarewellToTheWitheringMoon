import numpy as np
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
import glfw
import time
import random
import keyboard as kbd
import json
import os

from rigidbody import *
from model import *
from graphics import *
from camera import *
from ui import *
from sound import *
from weapon import *

def main():
    
    def window_resize(window, width, height):
        try:
            # glfw.get_framebuffer_size(window)
            main_cam.unlock()
            glViewport(0, 0, width, height)
            glLoadIdentity()
            gluPerspective(fov, width/height, near_clip, far_clip)
            main_cam.orient = np.eye(3)
            glTranslate(main_cam.pos[0], main_cam.pos[1], main_cam.pos[2])
            main_cam.lock_to_target(player)
            
        except ZeroDivisionError:
            # if the window is minimized it makes height = 0, but we don't need to update projection in that case anyway
            pass

    # PLAYER
    player_model = Model("cigar")
    player_CoM = np.array([0, 0, 0])
    player_pos = np.array([0, 0, -20])
    player_vel = np.array([0, 0, 0])
    player_accel = np.array([0, 0, 0])
    player_orient = np.array([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 1]])
    player_ang_vel = np.array([0, 0, 0])
    player_ang_accel = np.array([0, 0, 0])
    player_mass = 1500e3
    player_inertia = np.array([[100e3, 0, 0],
                               [0, 100e3, 0],
                               [0, 0, 100e3]])
    player_max_thrust = player_mass * 9.81 * 0.2
    player_throttle_range = [0, 100]
    player_throttle = 0
    player_prop_mass = 3000e3
    player_mass_flow = 0.1e3
    player_cy_h = 300
    player_cy_r = 25
    player = Ship(player_model, player_CoM,
                  player_pos, player_vel, player_accel,
                  player_orient, player_ang_vel, player_ang_accel,
                  player_mass, player_inertia,
                  player_max_thrust, player_throttle_range, player_throttle,
                  player_prop_mass, player_mass_flow,
                  player_cy_h, player_cy_r)

    player_MG1 = Machinegun(player, np.array([25, 0, 100]),
                             1000, 0.12, 10000, 2)

    player_MG2 = Machinegun(player, np.array([25, 0, -100]),
                             1000, 0.12, 10000, 3)

    player_MG3 = Machinegun(player, np.array([-25, 0, 100]),
                             1000, 0.12, 10000, 4)

    player_MG4 = Machinegun(player, np.array([-25, 0, -100]),
                             1000, 0.12, 10000, 5)

    player.weapons = [player_MG1, player_MG2, player_MG3, player_MG4]

    enemy = Ship(player_model, player_CoM,
                 player_pos + np.array([100, 50, 10e3]), np.array([0, 0, -50]), player_accel,
                 player_orient, player_ang_vel, player_ang_accel,
                 player_mass, player_inertia,
                 player_max_thrust, player_throttle_range, player_throttle,
                 player_prop_mass, player_mass_flow,
                 player_cy_h, player_cy_r)

    objects = [player, enemy]
    player.set_target(enemy)

    stars = generateStars()
        
    # SOUND
    print("Initializing sound (pygame.mixer)...")
    init_sound()

    # GRAPHICS
    print("Initializing graphics (OpenGL, glfw)...")
    window_x, window_y = 1600, 900
    fov = 70
    near_clip = 0.5
    far_clip = 1e6
    
    glfw.init()
    window = glfw.create_window(window_x, window_y, "Farewell To The Withering Moon", None, None)
    glfw.set_window_pos(window, 100, 100)
    glfw.make_context_current(window)
    glfw.set_window_size_callback(window, window_resize)

    gluPerspective(fov, window_x/window_y, near_clip, far_clip)
    glClearColor(0, 0, 0, 1)

    # CAMERA
    cam_pos = np.array([0, 0, 0])
    cam_orient = np.array([[-1, 0, 0],
                           [0, 1, 0],
                           [0, 0, -1]])
    main_cam = Camera("main_cam", cam_pos, cam_orient, True)

    glRotate(180, 0, 1, 0)    
    main_cam.lock_to_target(player)
    main_cam.rotate([-10, 0, 0])

    def move_cam(movement):
        main_cam.move(movement)

    def rotate_cam(rotation):
        main_cam.rotate(rotation)

    # CAMERA CONTROLS
    cam_pitch_up = "K"
    cam_pitch_dn = "I"
    cam_yaw_left = "J"
    cam_yaw_right = "L"
    cam_roll_cw = "O"
    cam_roll_ccw = "U"
    cam_rot_speed = 30

    player_pitch_dn = "W"
    player_pitch_up = "S"
    player_yaw_left = "A"
    player_yaw_right = "D"
    player_roll_ccw = "Q"
    player_roll_cw = "E"
    player_throttle_up = "R"
    player_throttle_dn = "F"
    cam_move_speed = 10

    player_shoot = "Space"
    play_bgm("Combat")

    print("Starting...")
    print("= = = = = =\n")
    dt = 0
    game_running = True
    while not glfw.window_should_close(window):
        t_cycle_start = time.perf_counter()
        glfw.poll_events() 

        # CONTROLS
        # -- -- camera controls
        if kbd.is_pressed(cam_pitch_up):
            rotate_cam([cam_rot_speed * dt, 0, 0])
        if kbd.is_pressed(cam_pitch_dn):
            rotate_cam([-cam_rot_speed * dt, 0, 0])
        if kbd.is_pressed(cam_yaw_left):
            rotate_cam([0, cam_rot_speed * dt, 0])
        if kbd.is_pressed(cam_yaw_right):
            rotate_cam([0, -cam_rot_speed * dt, 0])
        if kbd.is_pressed(cam_roll_cw):
            rotate_cam([0, 0, cam_rot_speed * dt])
        if kbd.is_pressed(cam_roll_ccw):
            rotate_cam([0, 0, -cam_rot_speed * dt])

        # -- -- player rotation
        if kbd.is_pressed(player_pitch_dn):
            player.apply_torque(np.array([-4000, 0, 0]))
        if kbd.is_pressed(player_pitch_up):
            player.apply_torque(np.array([4000, 0, 0]))
        if kbd.is_pressed(player_yaw_right):
            player.apply_torque(np.array([0, 4000, 0]))
        if kbd.is_pressed(player_yaw_left):
            player.apply_torque(np.array([0, -4000, 0]))
        if kbd.is_pressed(player_roll_ccw):
            player.apply_torque(np.array([0, 0, -12000]))
        if kbd.is_pressed(player_roll_cw):
            player.apply_torque(np.array([0, 0, 12000]))

        # -- -- player translation
        if kbd.is_pressed(player_throttle_up):
            player.set_thrust_percent(player.throttle + 15 * dt)
        elif kbd.is_pressed(player_throttle_dn):
            player.set_thrust_percent(player.throttle - 15 * dt)

        # -- -- player offensive
        if kbd.is_pressed(player_shoot):
            for w in player.weapons:
                w.shoot(objects)

        # PHYSICS
        player.apply_thrust()
        player.drain_fuel(dt)
        player.update(dt)

        for pw in player.weapons:
            pw.update(dt)

        for o in objects:
            o.update(dt)

            if isinstance(o, Bullet):
                o.update_timer(objects, dt)
                collider = o.check_collision(objects)
                if collider:
                    objects.remove(o)
                    del o

        main_cam.move_with_lock()
                
        # GRAPHICS
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        drawScene(main_cam, player, objects, stars)

        throttle_text = "Throttle:" + str(int(player.throttle))
        render_AN(throttle_text, Color(0,0,1), [-10, 6.5], main_cam)

        vel_text = "VX:" + str(int(player.vel[0])) + " VY:" + str(int(player.vel[1])) +  " VZ:" + str(int(player.vel[2]))
        render_AN(vel_text, Color(0,0,1), [-10, 6], main_cam)

        proj_text = ""
        for w in player.weapons:
            if isinstance(w, Machinegun):
                proj_text += str(w.num_projectiles) + "\n"

        render_AN(proj_text, Color(1,0,0), [-10, 5.5], main_cam, font_size=0.05)
        
        glfw.swap_buffers(window)
        dt = time.perf_counter() - t_cycle_start
        
    glfw.destroy_window(window)
    fade_out_bgm(2000)
    print("Quitting...")
    time.sleep(2)

main()
