# ----- level 1 - Pilote Manuel ---- #
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
    # Complète la fonction pour controller le drone

    keyboard_inputs = controller.get_keyboard_inputs() # retourne un dictionnaire [Clé = Nom de la touche] [Valeur = True / False], pour lire les entrées clavier

    # ETAPE 1 : Si aucune touche n'est appuyé -> on égalise les puissance moteurs à la puissance moyenne entre les 2 moteurs
    # utilise m.setLeftPower() / m.setRightPower() pour mettre les moteurs à la puissance moyenne
    moyenne_moteurs = (m.getLeftPower() + m.getRightPower()) / 2

    # ETAPE 2 : Si on appui sur les flèche haut ou bas -> augmente ou diminue la puissance des deux moteurs
    # utilise m.increaseLeftPower() et m.increaseRightPower()
    if keyboard_inputs['UP']:
        # augmente la puissance des moteurs
        pass
    if keyboard_inputs['DOWN']:
        # diminue la puissance des moteurs
        pass

    # ETAPE 3 : fait pivoté le drone sans changé la puissance totale
    # utilise m.increaseLeftPower() et m.increaseRightPower()
    if keyboard_inputs['RIGHT']:
        # Pour cela on peut augmenté le moteur gauche et diminué le moteur droit
        pass

    # Flèche de gauche -> pareil qu'à droite mais en inversé
    if keyboard_inputs['LEFT']:
        pass

    return

machine.set_update_function(drone_control_function)
controller.StartSimulation(LEVEL)
