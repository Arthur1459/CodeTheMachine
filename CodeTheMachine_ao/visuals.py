import pygame.image as pgi
import pygame.transform as pgt
import os

def get_path(path):
    return os.path.join(os.path.dirname(__file__), path)
def resize(img, tuple_size):
    return pgt.scale(img, tuple_size)

drone = pgi.load(get_path("rsc/visuals/machines/drone/drone.png"))
car = pgi.load(get_path("rsc/visuals/machines/car/car.png"))
energy = pgi.load(get_path("rsc/visuals/collectables/energy.png"))
orb = pgi.load(get_path("rsc/visuals/collectables/orb.png"))

