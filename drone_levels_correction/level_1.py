# ------------- level -------------- #
LEVEL = 1
# ---------------------------------- #

# ------------ Objectif ------------ #
# Faire décoller le drone
# ---------------------------------- #

# Importe le "controller" du drone simulé
# Ce controller permet de configurer la simulation, ainsi que de controller la machine
import CodeTheMachine_ao.controller as controller

# Initialise une machine à controler, ici un drone
machine = controller.LoadMachine(machine_type="drone")

# Défini la fonction de controle du drone
def drone_control_function(m: controller.machine.Drone) -> None:
    # A chaque pas de simulation le drone est "mise à jour" et cette fonction est appelée
    # Tu peux donc définir la puissance à mettre dans les moteurs droit et gauche
    # les fonctions utiles sont :
    #
    # m.setLeftPower(value in %)
    # m.setRightPower(value in %)
    #
    # ex: mets les 2 moteurs à 5% de puissance
    m.setLeftPower(20)
    m.setRightPower(20)
    return

# Informe au drone la fonction du control à utiliser (ne pas changé)
machine.set_update_function(drone_control_function)

# Lance la simulation
controller.StartSimulation(LEVEL)
