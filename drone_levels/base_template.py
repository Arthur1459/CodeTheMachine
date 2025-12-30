# ----------- CodeTheMachine - Drone Template ---------------- #
#
# Here you can find the feature to setup and custom the simulation.
#

import CodeTheMachine_ao.controller as control

# Load Machine
machine = control.LoadMachine(machine_type="drone")

# Colliders (exemple)
control.addWall(control.Wall((350, 150), (350, 300), 20))

# Collectable (exemple)
control.addCollectable(control.Orb((200, 200)))
control.addCollectable(control.Energy((500, 200), respawn_time=10))
control.addCollectable(control.Timer((650, 200), duration=12, reset_score=True))

# Slider (exemple)
slider_exemple = control.addSlider("slider exemple", 0, 10, 0, (600, 400), vertical=False, length=200)

# Drone Update function
def update(m: control.machine.Drone):

    # Read Slider value
    global slider_exemple
    print("Slider : ", slider_exemple.getValue())

    # key available : UP, DOWN, LEFT, RIGHT, SPACE, A, Z, E, R, T, Y
    keyboard_inputs = control.get_keyboard_inputs()
    if keyboard_inputs["UP"]:
        print("UP !")
        m.setLeftPower(25)
        m.setRightPower(25)
    else:
        m.setLeftPower(0)
        m.setRightPower(0)

    # Draw a redpoint on the simulation at the coordinate
    control.drawPoint((400, 400))

# Set the amount of energy lost in %
control.setEnergyLoss(100)

# Set update function and start simulation
machine.set_update_function(update)
control.StartSimulation()
