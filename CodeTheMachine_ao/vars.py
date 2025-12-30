# Global variables used by the game
import CodeTheMachine_ao.config as cf

window_size = cf.window_default_size
win_width, win_height = window_size[0], window_size[1]
middle = (win_width // 2, win_height // 2)

window = None
clock = None

running = False
pause_sim = False

# In game
inputs = {}
fps = cf.fps
dt, t, t_start = 0.3, 0, 0

cursor = (0, 0)
info_txt = ""
id = 0

machine_type: None
machine, machine_type = None, None
colliders = []
collectables = []
gui = []
perturbations = []

energy_loss = cf.base_energy_loss
score = 0
level, level_succeeded, time_succeeded = None, False, -1
