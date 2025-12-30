import pygame as pg

import CodeTheMachine_ao.tools as t
import CodeTheMachine_ao.utils as u
import CodeTheMachine_ao.vars as vr
import CodeTheMachine_ao.config as cf
import CodeTheMachine_ao.vector as v
import CodeTheMachine_ao.visuals as visuals

class _Circle:
    def __init__(self, center, radius):
        super().__init__()

        self.center = center if isinstance(center, v.Vector) else v.Vector(*center)
        self.radius = radius

    def collide(self, point: v.Vector, margin=0) -> bool:
        return t.distance(self.center, point) < self.radius + margin

    def draw(self):
        pg.draw.circle(vr.window, 'black', self.center(), self.radius)

class CircleComposedCollider:
    def __init__(self):
        self.id = u.getNewId()
        self.circles_elements = []
    def collide(self, point: v.Vector, margin=0):
        for c in self.circles_elements:
            if c.collide(point, margin=margin):
                return True, c
        return False, None
    def draw(self):
        for c in self.circles_elements:
            c.draw()

class Circle(CircleComposedCollider):
    def __init__(self, center: v.Vector, radius: int | float):
        super().__init__()

        self.center = center
        self.radius = radius

        self.set_colliders()

    def set_colliders(self):
        self.circles_elements = [_Circle(self.center, self.radius)]


class Wall(CircleComposedCollider):
    def __init__(self, start: v.Vector, end: v.Vector, width: int | float):
        super().__init__()

        self.start = start
        self.end = end
        self.width = width

        self.set_colliders()

    def set_colliders(self):
        length = t.distance(self.start, self.end)
        rapport = length / self.width
        nb_colliders = 3 * int(rapport)
        radius = length / int(rapport)
        for i in range(nb_colliders):
            self.circles_elements.append(_Circle(self.start + ((i + 0.5) / nb_colliders) * (self.end - self.start), radius))

