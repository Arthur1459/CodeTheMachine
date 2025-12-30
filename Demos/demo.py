import CodeTheMachine_ao.controller as control

machine = control.LoadMachine(machine_type="drone")

slider_kx_p = control.addSlider("kx_p", 0, 20, 15, control.Vector(10, 40))
slider_kx_d = control.addSlider("kx_d", 0, -50, -35, control.Vector(10, 60))
slider_kx_ia = control.addSlider("kx_i_angle", 0, 0.05, 0.001, control.Vector(10, 80))
slider_kx_ix = control.addSlider("kx_i_x", 0, 0.05, 0.001, control.Vector(10, 100))

slider_perturbation = control.addSlider("perturbation", 0., 1., 0., control.Vector(10, 140))
def perturbation():
    global slider_perturbation
    return control.Vector(slider_perturbation.getValue(), 0)
control.addPerturbation(perturbation) # A force toward +x (in g)

trajectory = [control.Vector(control.config.window_x_size/2 - 0, control.config.window_y_size/2 - 0),
              control.Vector(control.config.window_x_size/2 - 400, control.config.window_y_size/2 - 0),
              control.Vector(control.config.window_x_size/2 - 400, control.config.window_y_size/2 - 300),
              control.Vector(control.config.window_x_size/2 - 0, control.config.window_y_size/2 - 300)]
checkpoint_index = 0
angle_objective = 0
Y_objective = None
X_objective = None
speedY_objective, speedY_max = 0, 100
speedX_objective, speedX_max = 0, 100
integral_X, integral_X_max = 0, 100
integral_Y, integral_Y_max = 0, 100

# Built-In PID
def control_drone_builtin_PID(m: control.machine.Drone):

    inputs = control.get_keyboard_inputs()
    if inputs['A']: control.reset()

    target = control.get_cursor()
    control.drawPoint(target)

    m.builtin_PID(target)

    return

#Proportional-Integral-Derivative Y-coord / X-coord control
def control_drone_X_Y_PID(m: control.machine.Drone):
    global speedX_objective, speedX_max, speedY_objective, speedY_max, X_objective, Y_objective, integral_X, integral_X_max, integral_Y, integral_Y_max

    if X_objective is None: X_objective = m.getXposition()
    if Y_objective is None: Y_objective = m.getYposition()

    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        Y_objective += -5 # Y-axis inverted
    elif inputs['DOWN']:
        Y_objective += 5
    Y_objective = min(control.getEnvironmentSize().y, max(0, Y_objective)) # Saturation

    speedY_objective = ((Y_objective - m.getYposition()) / control.getEnvironmentSize().y) * speedY_max
    speedY_objective = min(speedY_max, max(-1 * speedY_max, speedY_objective)) # Saturation en +- speedY_max

    if inputs['RIGHT']:
        X_objective += 5
    elif inputs['LEFT']:
        X_objective += -5

    X_objective = min(control.getEnvironmentSize().x, max(0, X_objective)) # Saturation en +- speedY_max

    speedX_objective = ((X_objective - m.getXposition()) / control.getEnvironmentSize().x) * speedX_max
    speedX_objective = min(speedX_max, max(-1 * speedX_max, speedX_objective)) # Saturation en +- speedX_max

    control.drawPoints([control.vector.Vector(X_objective, Y_objective)])

    # X Stabilisation Correction
    angle_target = (speedX_objective / speedX_max) * 45
    angle_target = min(10., max(-10., angle_target))

    k = 1
    k_a = 0.5
    k_d = -7
    k_i = 0.3

    angle_proportional_factor = k_a * control.delta_angle(angle_target)
    integral_X = min(integral_X_max, max(-integral_X_max, integral_X + angle_proportional_factor))
    angle_integral_factor = k_i * integral_X
    angle_speed_factor = k_d * m.getANGLEspeed()

    raw_correction = k * (angle_proportional_factor + angle_speed_factor + angle_integral_factor)
    correction = min(1, max(-1, raw_correction))
    delta = min(20, max(abs(m.getRightPower() - m.getLeftPower()), 0.1), max(5, m.getRightPower(), m.getLeftPower(), 100 - m.getRightPower(), 100 - m.getLeftPower()))  # To keep the same total power
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(-1 * delta * correction)

    # Y Stabilisation Correction
    k = 1
    k_a = 0.3
    k_d = -0.03
    k_i = 0.05
    Y_proportional_factor = k_a * (m.getYspeed() - speedY_objective)
    integral_Y = min(integral_Y_max, max(-integral_Y_max, integral_Y + Y_proportional_factor))
    Y_integral_factor = k_i * integral_Y
    Y_speed_factor = k_d * abs(m.getRightPower() + m.getLeftPower()) * control.cos(control.radians(m.getANGLE()))
    raw_correction = k * (Y_proportional_factor + Y_speed_factor + Y_integral_factor)
    correction = min(1, max(-1, raw_correction))

    delta = 20  # power of increase
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(delta * correction)

    return

# Positioning control: Proportional-Derivative Y-coord and X-coord control
def control_drone_X_Y_PD(m: control.machine.Drone):
    global speedX_objective, speedX_max, speedY_objective, speedY_max, X_objective, Y_objective

    if X_objective is None: X_objective = m.getXposition()
    if Y_objective is None: Y_objective = m.getYposition()

    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        Y_objective += -5 # Y-axis inverted
    elif inputs['DOWN']:
        Y_objective += 5
    Y_objective = min(control.getEnvironmentSize().y, max(0, Y_objective)) # Saturation

    speedY_objective = ((Y_objective - m.getYposition()) / control.getEnvironmentSize().y) * speedY_max
    speedY_objective = min(speedY_max, max(-1 * speedY_max, speedY_objective)) # Saturation en +- speedY_max

    if inputs['RIGHT']:
        X_objective += 5
    elif inputs['LEFT']:
        X_objective += -5
    X_objective = min(control.getEnvironmentSize().x, max(0, X_objective)) # Saturation en +- speedY_max

    speedX_objective = ((X_objective - m.getXposition()) / control.getEnvironmentSize().x) * speedX_max
    speedX_objective = min(speedX_max, max(-1 * speedX_max, speedX_objective)) # Saturation en +- speedX_max

    control.drawPoints([control.vector.Vector(X_objective, Y_objective)])

    # X Stabilisation Correction
    if speedX_objective != 0:
        angle_target = (speedX_objective / speedX_max) * 60 # 60° -> degree max
    else:
        angle_target = -1 * m.getXspeed() / max(1, abs(m.getXspeed())) * min(90, (abs(m.getXspeed()) / speedX_max) * 90) # 60° Stop drifting
    k = 1
    k_a = 2
    k_d = -10
    angle_proportional_factor = k_a * control.delta_angle(angle_target)
    angle_speed_factor = k_d * m.getANGLEspeed()
    raw_correction = k * (angle_proportional_factor + angle_speed_factor)
    correction = min(1, max(-1, raw_correction))

    delta = min(max(abs(m.getRightPower() - m.getLeftPower()), 1), max(5, m.getRightPower(), m.getLeftPower(), 100 - m.getRightPower(), 100 - m.getLeftPower()))  # To keep the same total power
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(-1 * delta * correction)

    # Y Stabilisation Correction
    k = 0.5
    k_a = 10
    k_d = -0.2
    Y_proportional_factor = k_a * (m.getYspeed() - speedY_objective)
    Y_speed_factor = k_d * abs(m.getRightPower() + m.getLeftPower()) * control.cos(control.radians(m.getANGLE()))
    raw_correction = k * (Y_proportional_factor + Y_speed_factor)
    correction = min(1, max(-1, raw_correction))

    delta = 5  # power of increase
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(delta * correction)

    return

# Trajectory following: Proportional-Derivative Y-coord and Angle control
def control_drone_Angle_PD_trajectory(m: control.machine.Drone):

    global trajectory, checkpoint_index

    control.drawPoints(trajectory)
    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    target = trajectory[checkpoint_index]
    if control.tools.distance(machine.getPosition(), target) < 10:
        checkpoint_index = (checkpoint_index + 1) % len(trajectory)

    delta_position = target - machine.getPosition()

    speed_saturation = 1

    if delta_position.y < 1 and m.getYspeed() > -speed_saturation:
        m.increaseRightPower(0.5)
        m.increaseLeftPower(0.5)
    elif delta_position.y > -1 and m.getYspeed() < speed_saturation:
        m.increaseRightPower(-0.5 )
        m.increaseLeftPower(-0.5)

    angle_objective = 0
    if delta_position.x < 1 and m.getXspeed() > -speed_saturation:
        angle_objective = -5
    elif delta_position.x > -1 and m.getXspeed() < speed_saturation:
        angle_objective = 5

    delta_power = min(max(abs(m.getRightPower() - m.getLeftPower()), 5), 50)

    # Stabilisation Correction PD
    k = 2
    k_a = 1/180
    k_d = -0.25
    angle_factor = k_a * control.delta_angle(angle_objective)
    speed_factor = k_d * m.getANGLEspeed()

    raw_correction = k * (angle_factor + speed_factor)
    correction = min(1, max(-1, raw_correction))

    m.increaseLeftPower(delta_power * correction)
    m.increaseRightPower(-1 * delta_power * correction)

    return

# Proportional-Derivative Y-coord and Angle control
def control_drone_Y_Angle_PD(m: control.machine.Drone):
    global angle_objective, speedY_objective

    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        speedY_objective += -1 # Y-axis inverted
    elif inputs['DOWN']:
        speedY_objective += 1
    else:
        speedY_objective = 0
    speedY_objective = (speedY_objective / max(1, abs(speedY_objective))) * min(10, max(0, abs(speedY_objective)))

    if inputs['RIGHT'] or inputs['LEFT']:
        if inputs['RIGHT']:
            angle_objective += 5
        elif inputs['LEFT']:
            angle_objective += -5
        angle_objective = (angle_objective / max(1, abs(angle_objective))) * min(30, max(0, abs(angle_objective)))
    else:
        angle_objective = 0

    # Angle Stabilisation Correction
    k = 2
    k_a = 1/180
    k_d = -0.2
    angle_proportional_factor = k_a * control.delta_angle(angle_objective)
    angle_speed_factor = k_d * m.getANGLEspeed()
    raw_correction = k * (angle_proportional_factor + angle_speed_factor)
    correction = min(1, max(-1, raw_correction))

    delta = min(max(abs(m.getRightPower() - m.getLeftPower()), 5), 50) # To keep the same total power
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(-1 * delta * correction)

    # Y Stabilisation Correction
    k = 1
    k_a = 20
    k_d = -0.1
    Y_proportional_factor = k_a * (m.getYspeed() - speedY_objective)
    Y_speed_factor = k_d * abs(m.getRightPower() + m.getLeftPower()) * control.sin(control.radians(m.getANGLE()))
    raw_correction = k * (Y_proportional_factor + Y_speed_factor)
    correction = min(1, max(-1, raw_correction))

    delta = 2  # power of increase
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(delta * correction)

    return

# Proportional-Derivative Y-coord control, Advance Keyboard Angle control
def control_drone_Y_PD(m: control.machine.Drone):
    global speedY_objective

    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        speedY_objective += -1  # Y-axis inverted
    elif inputs['DOWN']:
        speedY_objective += 1
    else:
        speedY_objective = 0
    speedY_objective = (speedY_objective / max(1, abs(speedY_objective))) * min(10, max(0, abs(speedY_objective)))

    if inputs['RIGHT'] or inputs['LEFT']:
        if inputs['RIGHT']:
            right, left = m.getRightPower(), m.getLeftPower()
            m.setLeftPower(right + 10)
            m.setRightPower(left - 10)
        elif inputs['LEFT']:
            right, left = m.getRightPower(), m.getLeftPower()
            m.setLeftPower(right - 10)
            m.setRightPower(left + 10)
    else:
        m.setLeftPower(m.getRightPower())

    # Y Stabilisation Correction
    k = 1
    k_a = 20
    k_d = -0.1
    Y_proportional_factor = k_a * (m.getYspeed() - speedY_objective)
    Y_speed_factor = k_d * abs(m.getRightPower() + m.getLeftPower()) * control.sin(control.radians(m.getANGLE()))
    raw_correction = k * (Y_proportional_factor + Y_speed_factor)
    correction = min(1, max(-1, raw_correction))

    delta = 2  # power of increase
    m.increaseLeftPower(delta * correction)
    m.increaseRightPower(delta * correction)

# advanced keyboard power control (still hard to control)
def control_drone_advance(m: control.machine.Drone):
    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        m.increaseRightPower(1)
        m.increaseLeftPower(1)
    elif inputs['DOWN']:
        m.increaseRightPower(-1)
        m.increaseLeftPower(-1)

    if inputs['RIGHT'] or inputs['LEFT']:
        if inputs['RIGHT']:
            right, left = m.getRightPower(), m.getLeftPower()
            m.setLeftPower(right + 10)
            m.setRightPower(left - 10)
        elif inputs['LEFT']:
            right, left = m.getRightPower(), m.getLeftPower()
            m.setLeftPower(right - 10)
            m.setRightPower(left + 10)
    else:
        m.setLeftPower(m.getRightPower())

# keyboard power control (hard to control)
def control_drone_basic(m: control.machine.Drone):
    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        if inputs['RIGHT']:
            m.increaseLeftPower(0.1)
        elif inputs['LEFT']:
            m.increaseRightPower(0.1)
        else:
            m.increaseRightPower(0.2)
            m.increaseLeftPower(0.2)
    if inputs['DOWN']:
        if inputs['RIGHT']:
            m.increaseLeftPower(-0.1)
        elif inputs['LEFT']:
            m.increaseRightPower(-0.1)
        else:
            m.increaseRightPower(-0.2)
            m.increaseLeftPower(-0.2)

# just right & left motor
def rl_motor(m: control.machine.Drone):
    m.setLeftPower(70)
    m.setRightPower(50)

# just left the motor
def l_motor(m: control.machine.Drone):
    m.setLeftPower(50)

# Define the update function
machine.set_update_function(control_drone_builtin_PID_trajectory)

# Launch Simulation
control.StartSimulation()
print("Main Test Ended.")
