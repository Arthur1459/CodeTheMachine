# ------------- level -------------- #
LEVEL = 4
# ---------------------------------- #

# ------------ Objectif ------------ #
# Controller le drone de manière automatique pour qu'il suive un point objectif : correcteur proportionnel (+ Stabilisateur)
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

    point_objective = controller.get_target_point() # récupère les coordonées du point objectif (point_objective.x, point_objective.y)
    controller.drawPoint(point_objective) # affiche le point objectif sur la simulation

    # ------ Complète ici ------- #

    # ETAPE 1 : faire se diriger le drone vers le haut / bas en fonction de la hauteur de l'objectif
    if m.getHeightFromGround() < controller.distanceFromGround(point_objective.y): # si le drone est en dessous du point objectif (m.getHeightFromGround() -> Altitude du drone, controller.distanceFromGround(point_objective.y) -> Altitude du point objectif)
        # utilise m.setLeftPower ou m.setRightPower pour faire monter le drone (remarque : le drone décolle à partir de 20% de puissance dans les 2 moteurs)
        pass
    elif m.getHeightFromGround() >= controller.distanceFromGround(point_objective.y): # si le point objectif et au dessous du drone
        # utilise m.setLeftPower ou m.setRightPower pour faire descendre le drone
        pass

    # ETAPE 2 : Faire pivoter le drone dans la direction horizontale de l'objectif
    if m.getXposition() < point_objective.x: # si le point objectif et à droite du drone
        # utilise m.increaseLeftPower ou m.increaseRightPower créer une différence de puissance entre les moteur faisant tourner le drone vers la droite
        pass
    elif m.getXposition() > point_objective.x: # si le point objectif et à gauche du drone
        # utilise m.increaseLeftPower ou m.increaseRightPower créer une différence de puissance entre les moteur faisant tourner le drone vers la gauche
        pass

    # ETAPE 3 : Stabilise le drone en angle de roulie
    # tu peux changer "stabilisation_power" pour accroitre ou décroitre la puissance de la stabilisation
    # tu peux changer "angle_max" pour choisir à partir de quel angle tu active la stabilisation
    stabilisation_power = 1 # en % de puissance moteur
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
