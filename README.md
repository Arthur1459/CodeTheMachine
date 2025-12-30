# Code The Machine

### Simulate and control your machine !

#### Abstract 

Code The Machine aims to developed an easy-to-use framework to simulate different kind of vehicules / machine. \
The user can load machines et program them using there a controller which provide the access to the machine motors / sensors / ...

>For now, only the Drone Machine is fully available. Other still in dev.

## How to use it

Simply get the package `CodeTheMachine_ao` and then import the controller of the machine you want to simulate.

````
/src | --> /CodeTheMachine_ao
     | --> my_program.py
````

Ex 1 : Power only the left motor of drone 
```
import CodeTheMachine_ao.controller as control

machine = control.LoadMachine(machine_type="drone")

def power_left_motor(m: control.machine.Drone):
    m.setLeftPower(50)

machine.set_update_function(power_left_motor)

control.StartSimulation()
```

Ex 2 : Basic Control the drone with keyboard
```
import CodeTheMachine_ao.controller as control

machine = control.LoadMachine(machine_type="drone")

def control_drone_basic(m: control.machine.Drone):
    inputs = control.get_keyboard_inputs()

    if inputs['A']: control.reset()

    if inputs['UP']:
        if inputs['RIGHT']:
            m.increaseLeftPower(0.1)
        elif inputs['LEFT']:
            m.increaseRightPower(0.1)
        else:
            m.increaseRightPower(1)
            m.increaseLeftPower(1)
    if inputs['DOWN']:
        if inputs['RIGHT']:
            m.increaseLeftPower(-0.1)
        elif inputs['LEFT']:
            m.increaseRightPower(-0.1)
        else:
            m.increaseRightPower(-1)
            m.increaseLeftPower(-1)

machine.set_update_function(control_drone_basic)

control.StartSimulation()
```
Ex 3 : Use the Built-in PID controller to follow the cursor
```
import CodeTheMachine_ao.controller as control

machine = control.LoadMachine(machine_type="drone")

def control_drone_builtin_PID(m: control.machine.Drone):
    
    inputs = control.get_keyboard_inputs()
    if inputs['A']: control.reset()

    target = control.get_cursor()
    control.drawPoint(target)

    m.builtin_PID(target)

    return

machine.set_update_function(control_drone_builtin_PID)

control.StartSimulation()
```
>💡 It is possible to custom the PID parameter by passing them as argument : <br>
```
kx = {"global": 1., "p": 15, "d": -30, "i": 0.002}
ky = {"global": 1., "p": 0.5, "d": -0.08, "i": 0.03}
m.builtin_PID(target, kx=kx, ky=ky)
```