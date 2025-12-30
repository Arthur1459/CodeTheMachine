import os
import sys

import pygame as pg

import CodeTheMachine_ao.vars as vr

def resource_path(path):
    return os.path.join(os.path.dirname(__file__), path)

def Text(msg, coord, size, color):  # blit to the screen a text
    try:
        TextColor = pg.Color(color) # set the color of the text
        font = pg.font.Font(resource_path("rsc/pixel.ttf"), size)  # set the font
        return vr.window.blit(font.render(msg, True, TextColor), coord)  # return and blit the text on the screen
    except: pass

def getInputs():
    keys = pg.key.get_pressed()
    vr.inputs["SPACE"] = True if keys[pg.K_SPACE] else False

    vr.inputs["A"] = True if keys[pg.K_a] else False
    vr.inputs["Z"] = True if keys[pg.K_z] else False
    vr.inputs["E"] = True if keys[pg.K_e] else False
    vr.inputs["R"] = True if keys[pg.K_r] else False
    vr.inputs["T"] = True if keys[pg.K_t] else False
    vr.inputs["Y"] = True if keys[pg.K_y] else False

    vr.inputs["UP"] = True if keys[pg.K_UP] else False
    vr.inputs["DOWN"] = True if keys[pg.K_DOWN] else False
    vr.inputs["RIGHT"] = True if keys[pg.K_RIGHT] else False
    vr.inputs["LEFT"] = True if keys[pg.K_LEFT] else False

def isInWindow(coord):
    if 0 <= coord[0] <= vr.win_width:
        if 0 <= coord[1] <= vr.win_height:
            return True
    return False

def makeSeg(a, b):
    return lambda t: (b[0] + (t - 1) * (b[0] - a[0]), b[1] + (t - 1) * (b[1] - a[1]))

def cross_product(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]

def drawSeg(seg):
    pg.draw.line(vr.window, (20, 20, 100), seg(0), seg(1), 4)

def getNewId():
    vr.id += 1
    return vr.id

def angle_diff(angle_1, angle_2, rad=True):
    if rad:
        return (angle_1 - angle_2 + 3.14) % (2 * 3.14) - 3.14
    else:
        return (angle_1 - angle_2 + 180) % 360 - 180

def turn_angle(current, target, follow_strength):
    # Ensure angles are between 0 and 360
    current %= 360
    target %= 360
    # Compute shortest angular difference
    diff = (target - current + 180) % 360 - 180
    # Apply interpolation
    return (current + diff * follow_strength) % 360

def RotateIMG(image, angle, center, pivot_from_topleft) -> (pg.Surface, tuple[float, float]):
    # USAGE EXEMPLE :
    # visual, blit_pos = u.RotateIMG(self.visual, self.angle, self.position(), (0.5 * self.size)())
    # vr.window.blit(visual, blit_pos)
    #

    # offset from pivot to center
    image_rect = image.get_rect(topleft=(center[0] - pivot_from_topleft[0], center[1] - pivot_from_topleft[1]))
    offset_center_to_pivot = pg.math.Vector2(center) - image_rect.center

    # rotated offset from pivot to center
    rotated_offset = offset_center_to_pivot.rotate(angle) # Angle in DEGREES

    # rotated image center
    rotated_image_center = (center[0] - rotated_offset.x, center[1] - rotated_offset.y)

    # get a rotated image
    rotated_image = pg.transform.rotate(image, -angle)
    x_blit, y_blit, _, _ = rotated_image.get_rect(center=rotated_image_center)

    return rotated_image, (x_blit, y_blit)