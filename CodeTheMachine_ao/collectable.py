import pygame as pg

import CodeTheMachine_ao.tools as t
import CodeTheMachine_ao.utils as u
import CodeTheMachine_ao.vars as vr
import CodeTheMachine_ao.config as cf
import CodeTheMachine_ao.vector as v
import CodeTheMachine_ao.visuals as visuals
from CodeTheMachine_ao.vector import Vector

class Collectable:
    def __init__(self):
        self.id = u.getNewId()
        self.type_name = "Collectable"

        self.size: Vector = v.NullVector()
        self.position: Vector = v.NullVector()
        self.visual: pg.Surface = pg.Surface(self.size())

        self.collected: bool = False

    def isCollected(self) -> bool: return self.collected
    def setCollected(self, value=True): self.collected = value
    def reset(self):
        self.collected = False

    def update(self): return

    def draw(self):
        if not self.isCollected() and self.visual is not None:
            vr.window.blit(self.visual, self.position - self.size * 0.5)

    def isColliding(self, point: Vector) -> bool:
        return self.position.x - self.size.x/2 <= point.x <= self.position.x + self.size.x/2 and self.position.y - self.size.y/2 <= point.y <= self.position.y + self.size.y/2

class Energy(Collectable):
    def __init__(self, position, respawn_time=-1):
        super().__init__()
        self.type_name = "Energy"
        self.size = Vector(*cf.collectables_base_size)
        self.position = position if isinstance(position, Vector) else Vector(*position)
        self.visual = visuals.resize(visuals.energy, self.size)

        self.respawn_time = respawn_time
        self.respawnable = True if respawn_time > 0 else False
        self.last_collection_time = 0

    def setCollected(self, value=True):
        self.collected = value
        if value: self.last_collection_time = vr.t

    def update(self):
        if self.respawnable and self.isCollected() and vr.t - self.last_collection_time > self.respawn_time:
            self.setCollected(False)

class Orb(Collectable):
    def __init__(self, position, collected=False):
        super().__init__()
        self.setCollected(collected)
        self.type_name = "Orb"
        self.size = Vector(*cf.collectables_base_size)
        self.position = position if isinstance(position, Vector) else Vector(*position)
        self.visual = visuals.resize(visuals.orb, self.size)

class Timer(Collectable):
    def __init__(self, position, duration: float, reset_score=True):
        super().__init__()
        self.type_name = "timer"
        self.size = Vector(*cf.collectables_base_size)
        self.position = position if isinstance(position, Vector) else Vector(*position)
        self.visual = visuals.resize(visuals.orb, self.size)

        self.duration = duration
        self.start_time = 0
        self.started = False
        self.time_left = 0

        self.reset_score = reset_score

    def isFinished(self) -> bool:
        return self.time_left > 0

    def update(self):
        if self.isCollected() and not self.started:
            self.started = True
            self.start_time = vr.t

            if self.reset_score:
                vr.score = 0
                for c in vr.collectables:
                    if isinstance(c, Orb):
                        c.setCollected(False)

        if self.started :
            self.time_left = self.duration - (vr.t - self.start_time)
            if self.time_left <= 0 :
                self.started = False
                self.setCollected(False)

                if self.reset_score:
                    vr.score = 0
                    for c in vr.collectables:
                        if isinstance(c, Orb):
                            c.setCollected(True)
        else:
            self.time_left = 0

    def draw(self):
        if self.visual is not None:
            if self.started:
                pg.draw.rect(vr.window, "red", ((self.position - 0.5 * self.size)(), self.size()), 2)
                u.Text(f"{round(self.time_left, 1)}s", (self.position.x - 0.4 * self.size.x, self.position.y - 0.15 * self.size.y), 14, "red")
            else:
                pg.draw.rect(vr.window, "blue", ((self.position - 0.5 * self.size)(), self.size()), 2)
                u.Text(f"{round(self.duration, 1)}s",(self.position.x - 0.4 * self.size.x, self.position.y - 0.15 * self.size.y), 14, "blue")

    def reset(self):
        super().reset()
        self.started = False
