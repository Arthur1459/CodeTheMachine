import CodeTheMachine_ao.machine as mc
import CodeTheMachine_ao.machine as machine # used by user
import CodeTheMachine_ao.main_simulator as m
import CodeTheMachine_ao.levels_manager as lvls
import CodeTheMachine_ao.vars as _vr
import CodeTheMachine_ao.vector as vector
import CodeTheMachine_ao.config as config
import CodeTheMachine_ao.utils as u
import CodeTheMachine_ao.GUI as GUI
import CodeTheMachine_ao.tools as tools # may be used by user
from CodeTheMachine_ao.collectable import Collectable, Orb, Energy, Timer
from CodeTheMachine_ao.collider import Wall

from CodeTheMachine_ao.vector import Vector

from CodeTheMachine_ao.tools import radians, degrees, cos, sin

def LoadMachine(machine_type="default", position=None):

    if machine_type == "drone":
        _vr.machine = mc.Drone()
        _vr.machine.position = position if position is not None else Vector(config.window_x_size/2, config.window_y_size - lvls.getGroundHeight())
    elif machine_type == "car":
        _vr.machine = mc.Car()
        _vr.machine.position = position if position is not None else Vector(config.window_x_size / 2, config.window_y_size / 2)
    else:
        print("! No machine type specified !")
        _vr.machine = mc.Machine()

    _vr.machine_type = machine_type

    print("Machine Loaded : ", _vr.machine)
    return _vr.machine

def StartSimulation(level=None):
    lvls.load_lvl(level)
    m.main_simulation()

def StopSimulation():
    _vr.running = False

def addPerturbation(perturbation):
    if hasattr(perturbation, '__call__'):
        _vr.perturbations.append(perturbation)
    else: print("Error : perturbation must be callable (function that return a force as a Vector)")

def get_keyboard_inputs():
    return _vr.inputs

def get_cursor():
    return Vector(*_vr.cursor)

def get_target_point():
    return lvls.getPointObjective()

def delta_angle(angle):
    return u.angle_diff(angle, _vr.machine.angle, rad=False)

def reset():
    m.reset_sim()

def drawPoint(point):
    m.pg.draw.circle(_vr.window, 'red', point, 4)

def drawPoints(points):
    for p in points: m.pg.draw.circle(_vr.window, 'red', p, 4)

def getEnvironmentSize() -> vector.Vector: return vector.Vector(config.window_x_size, config.window_y_size)

def distanceFromGround(y_position):
    return config.window_y_size - lvls.getGroundHeight() - y_position

def addSlider(label, low, high, value_init, position, vertical=False, length=150):
    """
    Add a slider for the user to control the value of variables.
    :param label: Name displayed (string)
    :param low: Low value (int / float)
    :param high: High value (int / float)
    :param value_init: Initial value (int / float)
    :param position: Position the slider (Vector)
    :param vertical: (Optional) to have a vertical slider (bool) (False by default)
    :return: Slider object (read value with Slider.getValue())
    """
    if not isinstance(position, Vector): position = Vector(*position)
    slider = GUI.Slider(label, low, high, position, length, min(1, max(0, (value_init - low) / (high - low))), vertical=vertical)
    _vr.gui.append(slider)
    return slider

def addCollectable(c):
    if isinstance(c, Collectable):
        _vr.collectables.append(c)
    else:
        print("Error : object is not collectable.")

def addWall(w):
    if isinstance(w, Wall):
        _vr.colliders.append(w)
    else:
        print("Error : object is not a wall.")

def setEnergyLoss(value):
    """
    factor of the amount of energy lost each update in %
    :param value: in %
    :return: None
    """
    _vr.energy_loss = 0.01 * value * config.base_energy_loss