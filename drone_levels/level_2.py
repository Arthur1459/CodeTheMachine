# ------ level 1 - Aller-Retour ----- #
LEVEL = 2
# ---------------------------------- #

# ------------ Objectif ------------ #
# Faire décoller le drone jusqu'à mi-hauteur, puis le faire se reposer par terre
# ---------------------------------- #

import CodeTheMachine_ao.controller as controller
machine = controller.LoadMachine(machine_type="drone")

# Défini la fonction de controle du drone :
height_reached = False # Défini une variable d'etat utile "height_reached" (en français "hauteur_atteinte")

def drone_control_function(m: controller.machine.Drone) -> None:
    # les fonctions utiles sont :
    #
    # m.setLeftPower(value in %)
    # m.setRightPower(value in %)
    #
    # m.getHeightFromGround() -> renvoi la l'altitude du drone
    # controller.getEnvironmentSize().y -> renvoi la "hauteur" totale du monde
    #
    # Il va falloir utilisé une variable d'etat qui se souviens entre 2 appels de cette fonction si le drone a atteint la hauteur voulue
    global height_reached # Le mot-clé "Global" permet d'informer python de l'utilisation d'une variable défini HORS de la fonction, ici la variable d'etat "height_reached"

    # ex: mesure l'altitude du drone, la "print", et si cette altitude atteint la mi-hauteur, change l'etat de "height_reached" à True
    print("Position selon y : ", m.getHeightFromGround())
    print("Hauteur atteinte ? : ", height_reached)
    if m.getHeightFromGround() > 0.5 * controller.getEnvironmentSize().y:
        height_reached = True

    # Code ici : (il va falloir implémenter une condition..)

    return

# Informe au drone la fonction du control à utilise et Lance la simulation
machine.set_update_function(drone_control_function)
controller.StartSimulation(LEVEL)
