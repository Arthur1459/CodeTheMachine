import pygame as pg
import time
from math import cos, sin, pi, tan

import CodeTheMachine_ao.tools as t
import CodeTheMachine_ao.utils as u
import CodeTheMachine_ao.vars as vr
import CodeTheMachine_ao.config as cf
import CodeTheMachine_ao.levels_manager as lvls

def init_sim():

    pg.init()
    pg.display.set_caption(cf.game_name)

    # screen initialisation
    if not cf.fullscreen:
        vr.window = pg.display.set_mode(vr.window_size)
    else:
        vr.window = pg.display.set_mode((0, 0), pg.FULLSCREEN)
        vr.window_size = vr.window.get_size()

    vr.clock = pg.time.Clock()
    vr.inputs["MOUSE_PRESSED"] = False
    return

def main_simulation(level=None):
    init_sim()

    vr.running = True
    frames_fps, t_fps, vr.t_start = 0, 0, time.time()

    print("Simulation started.")
    while vr.running:

        vr.clock.tick(cf.fps)

        vr.t = time.time() - vr.t_start
        frames_fps += 1
        vr.fps = frames_fps * t.inv(vr.t - t_fps)
        if frames_fps > 1000:
            frames_fps, t_fps = 0, time.time() - vr.t_start

        for event in pg.event.get():
            if event.type == pg.QUIT:
                vr.running = False
            elif event.type == pg.MOUSEBUTTONDOWN:
                vr.inputs["MOUSE_PRESSED"] = True
                print("Cursor : ", pg.mouse.get_pos())
            elif event.type == pg.MOUSEBUTTONUP:
                vr.inputs["MOUSE_PRESSED"] = False
                print("Cursor : ", pg.mouse.get_pos())

        # Main Loop #
        pre_update()
        if vr.fps > cf.fps * cf.fps_treshold and not vr.pause_sim:
            u.getInputs()
            update()
        post_update()
        # --------- #

    print("Simulation Ended.")
    return

def update():
    vr.cursor = pg.mouse.get_pos()

    for collider in vr.colliders:
        collider.draw()

    for collectable in vr.collectables:
        collectable.update()
        collectable.draw()
        if vr.machine is not None:
            vr.machine.try_collect(collectable)

    if vr.machine is not None:
        vr.machine.update()
        vr.machine.draw()

    for gui_elt in vr.gui:
        gui_elt.update()
        gui_elt.draw()

    if lvls.isLevelSucceeded() and not vr.level_succeeded:
        vr.level_succeeded = True
        vr.time_succeeded = vr.t

    return

def pre_update():
    vr.window.fill('lightblue')

def post_update():
    pg.draw.rect(vr.window, "black", pg.rect.RectType(vr.win_width - 88, 2, 85, 20))
    u.Text("fps : " + str(round(vr.fps, 1)), (vr.win_width - 84, 5), 12, 'white')
    u.Text("Score : " + str(vr.score), (vr.win_width * 0.5 - 10, 10), 18, 'black')
    if vr.level_succeeded:
        u.Text(f"Level Succeeded ! ({round(vr.time_succeeded, 2)}s)", (vr.win_width * 0.3 - 10, vr.win_height*0.5 - 5), 28, "yellow")
    pg.display.update()
    return

def reset_sim():
    vr.score = 0
    vr.level_succeeded, vr.time_succeeded = False, -1
    for collectable in vr.collectables:
        collectable.reset()
    if vr.machine is not None:
        vr.machine.reset()
