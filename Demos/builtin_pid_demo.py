import CodeTheMachine_ao.controller as control

from CodeTheMachine_ao.vector import *
import CodeTheMachine_ao.config as config

# Load a Machine
machine = control.LoadMachine(machine_type="drone")

# Setup Slider vor controlling variables
slider_kx_p = control.addSlider("kx_p", 0, 20, 0., Vector(config.window_x_size - 165, 40), length=120)
slider_kx_d = control.addSlider("kx_d", 0, -50, 0., Vector(config.window_x_size - 165, 70), length=120)
slider_kx_i = control.addSlider("kx_i", 0, 0.05, 0., Vector(config.window_x_size - 165, 100), length=120)

slider_ky_p = control.addSlider("ky_p", 0, 1, 0.3, Vector(config.window_x_size - 165, 180), length=120)
slider_ky_d = control.addSlider("ky_d", 0, -0.1, -0.03, Vector(config.window_x_size - 165, 210), length=120)
slider_ky_i = control.addSlider("ky_i", 0, 0.1, 0.05, Vector(config.window_x_size - 165, 240), length=120)

slider_x_perturbation = control.addSlider("perturbation_x", 0., 1., 0., Vector(config.window_x_size - 165, 290), length=120)
slider_y_perturbation = control.addSlider("perturbation_y", 0., 5., 0., Vector(config.window_x_size - 165, 320), length=120)

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

# Define a perturbation function (return the vector x tq F = x * g with g the gravity intensity)
def perturbation():
    global slider_x_perturbation, slider_y_perturbation
    return Vector(slider_x_perturbation.getValue(), slider_y_perturbation.getValue())
control.addPerturbation(perturbation)

# Define points that the drone will try to follow
trajectory = [Vector(config.window_x_size/2 + 150, config.window_y_size/2 - 0),
              Vector(config.window_x_size/2 - 350, config.window_y_size/2 - 0),
              Vector(config.window_x_size/2 - 350, config.window_y_size/2 - 150),
              Vector(config.window_x_size/2 - 350, config.window_y_size/2 - 300),
              Vector(config.window_x_size/2 + 150, config.window_y_size/2 - 300),
              Vector(config.window_x_size/2 + 150, config.window_y_size/2 - 150),]

# Vertical
#trajectory = [Vector(config.window_x_size/2 - 350, config.window_y_size/2 - 0),
#             Vector(config.window_x_size/2 - 250, config.window_y_size/2 - 300)]

# Horizontal
#trajectory = [Vector(config.window_x_size/2 - 350, config.window_y_size/2 - 0),
#              Vector(config.window_x_size/2 + 100, config.window_y_size/2 - 50)]

# 'Circle'
#trajectory = [Vector(config.window_x_size/2 - 350, config.window_y_size/2 - 100),
#              Vector(config.window_x_size/2 + 50, config.window_y_size/2 - 350),
#              Vector(config.window_x_size/2 + 200, config.window_y_size/2 - 150),
#              Vector(config.window_x_size/2 + 0, config.window_y_size/2 + 100)]


# Initialise some state / objective variable
checkpoint_index = 0

Angle_objective = 0
Y_objective = None
X_objective = None

speedY_objective, speedY_max = 0, 100
speedX_objective, speedX_max = 0, 100

integral_X, integral_X_max = 0, 100
integral_Y, integral_Y_max = 0, 100

# Trajectory following : using Buildtin PID control
def control_drone_builtin_PID_trajectory(m: control.machine.Drone):

    global trajectory, checkpoint_index
    global slider_kx_p, slider_kx_i, slider_kx_d
    global slider_ky_p, slider_ky_i, slider_ky_d

    control.drawPoints(trajectory)
    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    target = trajectory[checkpoint_index]
    if control.tools.distance(machine.getPosition(), target) < m.radius/4:
        checkpoint_index = (checkpoint_index + 1) % len(trajectory)

    kx = {"global": 1., "p": slider_kx_p.getValue(), "d": slider_kx_d.getValue(), "i": slider_kx_i.getValue()}
    ky = {"global": 1., "p": slider_ky_p.getValue(), "d": slider_ky_d.getValue(), "i": slider_ky_i.getValue()}
    m.builtin_PID(target, kx=kx, ky=ky, delta_max=15, angle_max=45, print_corr=False, xspeed_damp=True)

    return

# Define the update function
machine.set_update_function(control_drone_builtin_PID_trajectory)
#machine.set_pos_uncertainty(25)
#machine.set_speed_uncertainty(5)
machine.pid_speedY_max = 50

# Set the initial PID parameters
set_xpid_sliders(15, -30, 0.002)
set_ypid_sliders(0.5, -0.08, 0.03)

# Launch Simulation
control.StartSimulation()
print("Main Test Ended.")
