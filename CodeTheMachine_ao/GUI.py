import pygame as pg

import CodeTheMachine_ao.tools as t
import CodeTheMachine_ao.utils as u
import CodeTheMachine_ao.vars as vr
import CodeTheMachine_ao.config as cf
import CodeTheMachine_ao.vector as v

from CodeTheMachine_ao.vector import Vector

class Slider:
    def __init__(self, label, low, high, topleft: Vector, length: float, slider_pos=None, vertical=False, button_radius=None):
        self.id = u.getNewId()

        self.label = label
        self.high_value = high
        self.low_value = low

        self.vertical = vertical
        self.start: Vector = topleft
        self.end: Vector = self.start + length * (Vector(1, 0) if not self.vertical else Vector(0, 1))
        self.slider_pos: float = slider_pos if slider_pos is not None else 0

        self.button_radius = button_radius if button_radius is not None else length / 16

        self.overed_by_cursor = False

    def getPos(self, percent=None) -> Vector:
        if percent is None:
            return self.start + self.slider_pos * (self.end - self.start)
        else:
            return self.start + percent * (self.end - self.start)

    def setValue(self, value):
        self.slider_pos = max(0, min(1, (value - self.low_value) / (self.high_value - self.low_value)))

    def getValue(self):
        return self.low_value + self.slider_pos * (self.high_value - self.low_value)

    def draw(self):
        u.Text(f"{self.label}={round(self.getValue(), 3)}" , self.getPos(0.) + Vector(0, -3 * int(self.button_radius)), max(10, int(self.button_radius)), "black")
        pg.draw.line(vr.window, "red", self.start(), self.end(), int(self.button_radius/2))
        pg.draw.circle(vr.window, "black", self.getPos(), self.button_radius)
        if self.overed_by_cursor:
            pg.draw.circle(vr.window, "yellow", self.getPos(), self.button_radius, 1)

    def update(self):
        if t.distance(vr.cursor, self.getPos()) < self.button_radius:
            self.overed_by_cursor = True
        else:
            self.overed_by_cursor = False

        if self.overed_by_cursor and vr.inputs["MOUSE_PRESSED"]:
            if not self.vertical:
                self.slider_pos = max(0, min(1, (Vector(*vr.cursor).x - self.start.x) / (self.end.x - self.start.x)))
            else:
                self.slider_pos = max(0, min(1, (Vector(*vr.cursor).y - self.start.y) / (self.end.y - self.start.y)))
