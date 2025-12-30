# ------------- level -------------- #
LEVEL = 4
# ---------------------------------- #

# ------------ Objectif ------------ #
# Controller le drone de manière automatique pour qu'il suive un point objectif
# ---------------------------------- #

import CodeTheMachine_ao.controller as controller
machine = controller.LoadMachine(machine_type="drone")

# Défini la fonction de controle du drone :
def drone_control_function(m: controller.machine.Drone) -> None:
    # les fonctions utiles sont :
    #
    # m.increaseLeftPower(value in %)
    # m.increaseRightPower(value in %)
    #
    # m.setLeftPower(value in %)
    # m.setRightPower(value in %)
    #
    # m.getAngleRelativeToVertical() -> renvoi l'angle de roulie mesuré du drone par rapport à l'axe vertical (penche de 5° à gauche -> -5°  |  penche de 5° à droite -> 5°)
    # m.getXposition() -> renvoi la position horizontale du drone
    #
    # point_objective.y | point_objective.x -> renvoi les coordonnées x/y du point objectif (attention l'axe y est orienté du haut vers le bas)
    # controller.distanceFromGround(point_objective.y) -> renvoi l'altitude du point
    #

    point_objective = controller.get_target_point() # récupère les coordonées du point objectif
    controller.drawPoint(point_objective) # affiche le point sur la simulation

    # ------ Correction ------- #

    # Fait se diriger le drone vers le haut / bas en fonction de la hauteur de l'objectif
    if m.getHeightFromGround() < controller.distanceFromGround(point_objective.y):
        m.setLeftPower(30)
        m.setRightPower(30)
    elif m.getHeightFromGround() >= controller.distanceFromGround(point_objective.y):
        m.setLeftPower(10)
        m.setRightPower(10)

    # Fait pivoter le drone dans la direction horizontale de l'objectif
    if m.getXposition() < point_objective.x:
        m.increaseLeftPower(1)
        m.increaseRightPower(-1)
    elif m.getXposition() > point_objective.x:
        m.increaseLeftPower(-1)
        m.increaseRightPower(1)

    # Force le drone à se stabiliser
    stabilisation_power = 3  # en % de puissance moteur
    angle_max = 5  # en degrés

    if m.getAngleRelativeToVertical() > angle_max:
        m.increaseLeftPower(-1 * stabilisation_power)
        m.increaseRightPower(1 * stabilisation_power)
    if m.getAngleRelativeToVertical() < -1 * angle_max:
        m.increaseLeftPower(1 * stabilisation_power)
        m.increaseRightPower(-1 * stabilisation_power)

    # ------------------------- #

    return

machine.set_update_function(drone_control_function)
controller.StartSimulation(LEVEL)
