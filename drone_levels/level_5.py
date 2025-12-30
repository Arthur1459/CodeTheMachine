# ------------- level -------------- #
LEVEL = 5
# ---------------------------------- #

# ------------ Objectif ------------ #
# Paramètre le drone pour suivre un point objectif dans un environement complex et récupère les checkpoints dans le temps imparti.
# Expérimente l'influence et règle les différents paramètres du Controller PID (Proportionnel - Dérivé - Intégral) pour suivre efficacement le point objectif
# Aucun Code n'est à changé !
# ---------------------------------- #

import CodeTheMachine_ao.controller as controller
from CodeTheMachine_ao.controller import Vector, config
machine = controller.LoadMachine(machine_type="drone")

# Initialise des "Slider" qui permettent de controller les paramètres du PID
slider_kx_p = controller.addSlider("Proportionnel X", 0, 10, 0., Vector(config.window_x_size - 165, 60), length=120)
slider_kx_d = controller.addSlider("Derivé X", 0, -10, 0., Vector(config.window_x_size - 165, 90), length=120)
slider_kx_i = controller.addSlider("Integral X", 0, 0.05, 0., Vector(config.window_x_size - 165, 120), length=120)

slider_ky_p = controller.addSlider("Proportionnel Y", 0, 1, 0., Vector(config.window_x_size - 165, 200), length=120)
slider_ky_d = controller.addSlider("Derivé Y", 0, -0.1, 0., Vector(config.window_x_size - 165, 230), length=120)
slider_ky_i = controller.addSlider("Integral Y", 0, 0.1, 0., Vector(config.window_x_size - 165, 260), length=120)

# Défini la fonction de controle du drone :
point_objective = controller.getEnvironmentSize() * 0.5 # Point objectif controllé par l'utilisateur (initialisé au centre)

def drone_control_function(m: controller.machine.Drone) -> None:

    global point_objective
    keyboard_inputs = controller.get_keyboard_inputs()

    if keyboard_inputs["R"]: # Reset simulation
        point_objective = controller.getEnvironmentSize() * 0.5
        controller.reset()

    point_speed = 3 # change the speed of the objective point you control
    if keyboard_inputs["UP"]: point_objective.y += -point_speed
    if keyboard_inputs["DOWN"]: point_objective.y += point_speed
    if keyboard_inputs["RIGHT"]: point_objective.x += point_speed
    if keyboard_inputs["LEFT"]: point_objective.x += -point_speed

    controller.drawPoint(point_objective)  # affiche le point sur la simulation

    kx = {"global": 1., "p": slider_kx_p.getValue(), "d": slider_kx_d.getValue(), "i": slider_kx_i.getValue()}
    ky = {"global": 1., "p": slider_ky_p.getValue(), "d": slider_ky_d.getValue(), "i": slider_ky_i.getValue()}

    m.builtin_PID(point_objective, kx=kx, ky=ky, delta_max=15, angle_max=45, print_corr=False, xspeed_damp=True)

    return

machine.set_update_function(drone_control_function)

def set_xpid_sliders(kx_p, kx_d, kx_i):
    global slider_kx_p, slider_kx_d, slider_kx_i
    slider_kx_p.setValue(kx_p)
    slider_kx_d.setValue(kx_d)
    slider_kx_i.setValue(kx_i)
def set_ypid_sliders(ky_p, ky_d, ky_i):
    global slider_ky_p, slider_ky_d, slider_ky_i
    slider_ky_p.setValue(ky_p)
    slider_ky_d.setValue(ky_d)
    slider_ky_i.setValue(ky_i)

controller.StartSimulation(LEVEL)
