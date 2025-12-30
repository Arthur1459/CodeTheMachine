import CodeTheMachine_ao.vars as vr
import CodeTheMachine_ao.config as cf
import CodeTheMachine_ao.vector as v
from CodeTheMachine_ao.collectable import Collectable
from CodeTheMachine_ao.vector import Vector

import CodeTheMachine_ao.collider as collide
import CodeTheMachine_ao.collectable as collect

from abc import ABC, abstractmethod

# ------- Globals ------- #
segment_index, segment_t = -1, 0
point = v.NullVector()
# ----------------------- #

# ----------- Function used in simulator ------------- #
def load_lvl(level=None):
    if vr.machine_type == "drone":
        return DroneLvlManager.load_lvl(level)
    elif vr.machine_type == "car":
        return CarLvlManager.load_lvl(level)
    else:
        return NoneLvlManager.load_lvl(level)

def isLevelSucceeded():
    if vr.machine_type == "drone":
        return DroneLvlManager.isLevelSucceeded()
    elif vr.machine_type == "car":
        return CarLvlManager.isLevelSucceeded()
    else:
        return NoneLvlManager.isLevelSucceeded()

def getPointObjective() -> v.Vector:
    if vr.machine_type == "drone":
        return DroneLvlManager.getPointObjective()
    elif vr.machine_type == "car":
        return CarLvlManager.getPointObjective()
    else:
        return NoneLvlManager.getPointObjective()

def getGroundHeight() -> float:
    if vr.machine_type == "drone":
        return DroneLvlManager.getGroundHeight()
    elif vr.machine_type == "car":
        return CarLvlManager.getGroundHeight()
    else:
        return NoneLvlManager.getGroundHeight()
# --------------------------------------------------- #

# -------------------------------- Trajectory ------------------------------------#
def computePointObjectiveOnPath(trajectory, segment_duration) -> v.Vector:
    global segment_index, segment_t, point
    if segment_index < 0:
        segment_index = 0 # init
        segment_t = vr.t

    p1, p2 = trajectory[segment_index], trajectory[(segment_index + 1) % len(trajectory)]
    if v.distance(p1, p2) < v.distance(p1, point): # p2 reached
        segment_index = (segment_index + 1) % len(trajectory)
        segment_t = vr.t
        p1, p2 = trajectory[segment_index % len(trajectory)], trajectory[(segment_index + 1) % len(trajectory)]

    t = (vr.t - segment_t) / segment_duration[segment_index]
    point = p1 * (1 - t) + p2 * t
    return point
# -----------------------------------------------------------------------------#


class AbstractLvlManager(ABC):
    @staticmethod
    @abstractmethod
    def load_lvl(level=None) -> bool: return True
    @staticmethod
    @abstractmethod
    def isLevelSucceeded() -> bool: return False
    @staticmethod
    @abstractmethod
    def getPointObjective() -> v.Vector: return v.NullVector()
    @staticmethod
    @abstractmethod
    def getGroundHeight() -> float: return 0.

class NoneLvlManager(AbstractLvlManager):
    @staticmethod
    def load_lvl(level=None) -> bool:
        vr.energy_loss = 0
        return True
    @staticmethod
    def isLevelSucceeded() -> bool: return False
    @staticmethod
    def getPointObjective() -> v.Vector: return v.NullVector()
    @staticmethod
    def getGroundHeight() -> float: return 0.

class DroneLvlManager(AbstractLvlManager):
    # --------- Level Loading ------- #
    @staticmethod
    def load_lvl(level=None) -> bool:
        global segment_index
        segment_index = -1

        if level is not None:
            vr.colliders = []
            vr.collectables = []
            vr.energy_loss = 0

        if level == 0:
            pass
        elif level == 1 :
            pass
        elif level == 2 :
            pass
        elif level == 3 :
            vr.energy_loss = 0.2 * cf.base_energy_loss
            vr.collectables.append(collect.Orb(v.Vector(300, 150)))
            vr.collectables.append(collect.Orb(v.Vector(700, 150)))
            vr.collectables.append(collect.Orb(v.Vector(700, 400)))
            vr.collectables.append(collect.Orb(v.Vector(300, 400)))

            vr.collectables.append(collect.Energy(v.Vector(300, 275)))
            vr.collectables.append(collect.Energy(v.Vector(700, 275)))
        elif level == 4 :
            vr.energy_loss = 0.4 * cf.base_energy_loss
            vr.collectables.append(collect.Orb(Vector(cf.window_x_size / 2 - 0, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300)))
            vr.collectables.append(collect.Orb(Vector(cf.window_x_size / 2 - 200, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300)))
            vr.collectables.append(collect.Orb(Vector(cf.window_x_size / 2 + 200, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300)))

        elif level == 5:
            vr.energy_loss = 0.4 * cf.base_energy_loss
            vr.colliders.append(collide.Wall(v.Vector(800, 0), v.Vector(800, 400), 20))
            vr.colliders.append(collide.Wall(v.Vector(800, 400), v.Vector(1000, 400), 20))

            vr.colliders.append(collide.Wall(v.Vector(150, 100), v.Vector(150, 400), 16))
            vr.colliders.append(collide.Wall(v.Vector(300, 0), v.Vector(300, 300), 16))
            vr.colliders.append(collide.Wall(v.Vector(150, 400), v.Vector(300, 400), 16))
            vr.colliders.append(collide.Wall(v.Vector(300, 400), v.Vector(300, cf.window_y_size - DroneLvlManager.getGroundHeight()), 16))

            vr.colliders.append(collide.Wall(v.Vector(450, 200), v.Vector(600, 200), 16))
            vr.colliders.append(collide.Wall(v.Vector(600, 0), v.Vector(600, 200), 16))

            vr.colliders.append(collide.Wall(v.Vector(450, 300), v.Vector(600, 300), 16))
            vr.colliders.append(collide.Wall(v.Vector(600, 300), v.Vector(600, cf.window_y_size - DroneLvlManager.getGroundHeight()), 16))

            vr.collectables.append(collect.Timer(v.Vector(500, 400), 30.0))

            vr.collectables.append(collect.Orb(v.Vector(220, 50), True))
            vr.collectables.append(collect.Orb(v.Vector(210, cf.window_y_size - DroneLvlManager.getGroundHeight() - 100), True))
            vr.collectables.append(collect.Orb(v.Vector(75, 250), True))
            vr.collectables.append(collect.Orb(v.Vector(480, 80), True))
            vr.collectables.append(collect.Orb(v.Vector(700, 80), True))
            vr.collectables.append(collect.Orb(v.Vector(900, cf.window_y_size - DroneLvlManager.getGroundHeight() - 90), True))

            vr.collectables.append(collect.Energy(v.Vector(525, 250)))

        elif level is None:
            pass
        else:
            raise AttributeError(f"Error : level {level} does not exist.")

        vr.level = level
        print(f"Level {level} loaded.")
        return True

    # --------- Level Succeed Check ----------#
    @staticmethod
    def isLevelSucceeded() -> bool:
        if vr.level == 0:
            return DroneLvlManager.check_level_0()
        elif vr.level == 1:
            return DroneLvlManager.check_level_1()
        elif vr.level == 2:
            return DroneLvlManager.check_level_2()
        elif vr.level == 3:
            return DroneLvlManager.check_level_3()
        elif vr.level == 4:
            return DroneLvlManager.check_level_4()
        elif vr.level == 5:
            return DroneLvlManager.check_level_5()
        else:
            return DroneLvlManager.check_level_None()

    @staticmethod
    def isScoreReached(target):
        return vr.score >= target

    @staticmethod
    def check_level_None() -> bool:
        return False

    @staticmethod
    def check_level_0() -> bool:
        return True

    @staticmethod
    def check_level_1() -> bool:
        return vr.machine.speed.y < -0.1

    @staticmethod
    def check_level_2() -> bool:
        if vr.score == 0:
            if vr.machine.position.y <= cf.window_y_size * 0.5:
                vr.score = 1
        if vr.score == 1:
            if vr.machine.position.y >= cf.window_y_size - DroneLvlManager.getGroundHeight() - vr.machine.size.y * 0.5:
                vr.score = 2
        return DroneLvlManager.isScoreReached(2)

    @staticmethod
    def check_level_3() -> bool:
        return DroneLvlManager.isScoreReached(4)

    @staticmethod
    def check_level_4() -> bool:
        if vr.score == 3:
            if vr.machine.getHeightFromGround(exact=True) == 0:
                vr.score += 1
        return DroneLvlManager.isScoreReached(4)

    @staticmethod
    def check_level_5() -> bool:
        return DroneLvlManager.isScoreReached(6)

    # ----- Point Target ----- #
    @staticmethod
    def getPointObjective() -> v.Vector:
        if vr.level == 4: return DroneLvlManager.PointObjectiveLvl4()
        else: return v.NullVector()

    @staticmethod
    def PointObjectiveLvl4():
        global segment_index, segment_t, point
        segment_duration = [2, 2, 4, 2, 2]
        trajectory = [Vector(cf.window_x_size / 2 - 0, cf.window_y_size - DroneLvlManager.getGroundHeight()),
                      Vector(cf.window_x_size / 2 - 0, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300),
                      Vector(cf.window_x_size / 2 - 200, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300),
                      Vector(cf.window_x_size / 2 + 200, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300),
                      Vector(cf.window_x_size / 2 - 0, cf.window_y_size - DroneLvlManager.getGroundHeight() - 300)]
        return computePointObjectiveOnPath(trajectory, segment_duration)

    @staticmethod
    def getGroundHeight() -> float:
        return 200

class CarLvlManager(AbstractLvlManager):
    @staticmethod
    def load_lvl(level=None) -> bool:
        global segment_index
        segment_index = -1
        vr.energy_loss = 0.1 * cf.base_energy_loss

        if level is not None:
            vr.colliders = []
            vr.collectables = []

        if level == 0:
            vr.colliders.append(collide.Wall(Vector(300, 250), Vector(600, 250), 20))
            vr.collectables.append(collect.Energy(Vector(450, 200), respawn_time=3))
            vr.collectables.append(collect.Orb(Vector(450, 300)))
        vr.level = level
        print(f"Level {level} loaded.")
        return True

    @staticmethod
    def isLevelSucceeded() -> bool: return False
    @staticmethod
    def getPointObjective() -> v.Vector:
        if vr.level == 1: return CarLvlManager.PointObjectiveLvl1()
        else: return v.NullVector()
    @staticmethod
    def PointObjectiveLvl1():
        segment_duration = [2, 2, 4, 2, 2]
        trajectory = [Vector(cf.window_x_size / 2 - 200, cf.window_y_size / 2 - 200),
                      Vector(cf.window_x_size / 2 + 200, cf.window_y_size / 2 - 200),
                      Vector(cf.window_x_size / 2 + 200, cf.window_y_size / 2 + 200),
                      Vector(cf.window_x_size / 2 - 200, cf.window_y_size / 2 + 200)]
        return computePointObjectiveOnPath(trajectory, segment_duration)
    @staticmethod
    def getGroundHeight() -> float: return 0.

# --------------------------------------------------------------------------------#