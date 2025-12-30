# ------------- level -------------- #
LEVEL = 3
# ---------------------------------- #

# ------------ Objectif ------------ #
# Controller le drone avec le clavier pour récupérer les points en améliorant le système de pilotage !
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
    #
    # ex : lis les entrées des flèches directionnelles du clavier et change les puissance moteurs (attention l'exemple ne produit rien de cohérent)
    keyboard_inputs = controller.get_keyboard_inputs()

    # ------ Correction ------- #

    # Si aucune touche n'est appuyé -> on égalise les puissance moteurs
    moyenne_moteurs = (m.getLeftPower() + m.getRightPower()) / 2
    m.setRightPower(moyenne_moteurs)
    m.setLeftPower(moyenne_moteurs)

    # Flèche du haut -> on augmente la puissance des 2 moteurs
    if keyboard_inputs['UP']:
        m.increaseLeftPower(0.5)
        m.increaseRightPower(0.5)

    # Flèche du bas -> on diminue la puissance des 2 moteurs
    if keyboard_inputs['DOWN']:
        m.increaseLeftPower(-0.5)
        m.increaseRightPower(-0.5)

    # Flèche de droite -> on augmente le moteur gauche et diminue le moteur droit
    if keyboard_inputs['RIGHT']:
        m.increaseRightPower(-1)
        m.increaseLeftPower(1)

    # Flèche de gauche -> pareil qu'à droite mais en inversé
    if keyboard_inputs['LEFT']:
        m.increaseRightPower(1)
        m.increaseLeftPower(-1)

    # ------------------------- #

    return

# Informe au drone la fonction du control à utiliser
machine.set_update_function(drone_control_function)

# Lance la simulation
controller.StartSimulation(LEVEL)
