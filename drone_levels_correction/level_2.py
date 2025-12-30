# ------------- level -------------- #
LEVEL = 2
# ---------------------------------- #

# ------------ Objectif ------------ #
# Faire décoller le drone jusqu'à mi-hauteur, puis le faire se reposer par terre
# ---------------------------------- #

# Importe le "controller" du drone simulé
# Ce controller permet de configurer la simulation, ainsi que de controller la machine
import CodeTheMachine_ao.controller as controller

# Initialise une machine à controler, ici un drone
machine = controller.LoadMachine(machine_type="drone")

# Défini la fonction de controle du drone :
height_reached = False # Défini une variable d'etat utile "height_reached" (en français "heuteur_atteinte")

def drone_control_function(m: controller.machine.Drone) -> None:
    # les fonctions utiles sont :
    #
    # m.setLeftPower(value in %)
    # m.setRightPower(value in %)
    #
    # m.getHeightFromGround() -> renvoi la l'altitude du drone
    # controller.getEnvironmentSize().y -> renvoi la "hauteur" totale du monde
    #
    # Il va falloir utilisé une variable d'etat qui se souviens entre 2 appels de cette fonction si le drone à atteint la hauteur voulu
    global height_reached # Le mot-clé "Global" permet d'informer python de l'utilisation d'une variable défini hors de la fonction, ici la variable d'etat "height_reached"
    # ex: mesure l'altitude du drone, la print, et si cette altitude atteint la mi-hauteur, change l'etat de "height_reached" à True
    print("Position selon y : ", m.getHeightFromGround())
    print("Hauteur atteinte ? : ", height_reached)

    if m.getHeightFromGround() > 0.5 * controller.getEnvironmentSize().y:
        height_reached = True

    # ------ Correction ------- #
    if height_reached is False:
        m.setLeftPower(25)
        m.setRightPower(25)
    else:
        m.setLeftPower(0)
        m.setRightPower(0)
    # ------------------------- #
    
    return

# Informe au drone la fonction du control à utiliser
machine.set_update_function(drone_control_function)

# Lance la simulation
controller.StartSimulation(LEVEL)
