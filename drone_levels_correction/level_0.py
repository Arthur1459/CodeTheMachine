# ------------- level -------------- #
LEVEL = 0
# ---------------------------------- #

# ------------ Objectif ------------ #
# Lancer la simulation
# ---------------------------------- #

# Importe le "controller" du drone simulé
# Ce controller permet de configurer la simulation, ainsi que de controller la machine
import CodeTheMachine_ao.controller as controller

# Initialise une machine à controler, ici un drone
machine = controller.LoadMachine(machine_type="drone")

# Lance la simulation
controller.StartSimulation(LEVEL)
