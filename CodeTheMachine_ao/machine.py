import pygame as pg

import CodeTheMachine_ao.tools as t
import CodeTheMachine_ao.utils as u
import CodeTheMachine_ao.vars as vr
import CodeTheMachine_ao.config as cf
import CodeTheMachine_ao.vector as v
import CodeTheMachine_ao.levels_manager as lvls
import CodeTheMachine_ao.visuals as visuals
import CodeTheMachine_ao.collectable as collect

class Machine:
    def __init__(self):
        self.id = u.getNewId()
        self.name = "Machine n°" + str(self.id)

        self.position = v.NullVector()
        self.speed = v.NullVector()
        self.angle = 0 # in degrees
        self.angle_speed = 0 # in degrees/s

        self.pos_uncertainty = 0
        self.speed_uncertainty = 0

        self.size = v.Vector(75, 75)
        self.radius = self.size.norm/2
        self.energy = 100

        self.update_function = lambda x: None

    def __str__(self):
        return "Machine n°" + str(self.id)

    def set_update_function(self, func):
        if callable(func):
            self.update_function = func
        else: print("ERROR in sef_update_function(), argument must be a function.")

    def set_pos_uncertainty(self, value): self.pos_uncertainty = value
    def set_speed_uncertainty(self, value): self.speed_uncertainty = value

    def apply_physics(self) -> None: return
    def apply_external_forces(self) -> None: pass

    def update(self):
        self.update_function(self)

        self.apply_physics()

        self.position += vr.dt * self.speed
        self.angle = u.turn_angle(self.angle, self.angle + vr.dt * self.angle_speed, 1)

        displacement_body = v.NullVector()
        margin = 0. * self.size.norm/2
        for collider in vr.colliders:
            collide_check, collider_element = collider.collide(self.position, margin=margin)
            if collide_check:
                gap = collider_element.center - self.position
                overlap = collider_element.radius - gap.norm + margin
                if overlap > displacement_body.norm:
                    displacement_body = gap.normalised() * (overlap * 0.2)
        self.position -= displacement_body

        if self.position.y + self.size.y/2 >= cf.window_y_size and self.speed.y > 0 : self.speed.y = 0
        if self.position.y - self.size.y/2 <= 0 and self.speed.y < 0: self.speed.y = 0
        if self.position.x + self.size.x/2 >= cf.window_x_size and self.speed.x > 0 : self.speed.x = 0
        if self.position.x - self.size.y/2 <= 0 and self.speed.x < 0: self.speed.x = 0

        self.position.x = min(cf.window_x_size, max(0, self.position.x))
        if self.position.y > cf.window_y_size - lvls.getGroundHeight() - self.getBottomOffset():
            self.position.y = cf.window_y_size - lvls.getGroundHeight() - self.getBottomOffset()
            self.speed.y = 0
        self.position.y = min(cf.window_y_size - lvls.getGroundHeight() - self.getBottomOffset(), max(0., self.position.y))

        self.log()

    def log(self): return
    def __str__(self): return self.name

    def _get_blit_position(self):
        return (self.position - 0.5 * self.size)()

    def draw(self):
        pg.draw.circle(vr.window, 'red', self._get_blit_position(), self.radius)
        if cf.debug: u.Text(str(self) + "| Position :" + str(self.position) + str(self.angle) + "| Speed :" + str(self.speed) + str(self.angle_speed), (10, 10),12, 'blue')

    def getEnergy(self): return self.energy

    def getXspeed(self): return self.speed.x + t.rndInt(-self.speed_uncertainty, self.speed_uncertainty)
    def getYspeed(self): return self.speed.y + t.rndInt(-self.speed_uncertainty, self.speed_uncertainty)
    def getANGLEspeed(self): return self.angle_speed # IN DEGREE / s

    def getPosition(self): return v.Vector(self.getXposition(), self.getYposition())
    def getXposition(self): return self.position.x + t.rndInt(-self.pos_uncertainty, self.pos_uncertainty)
    def getYposition(self): return self.position.y + t.rndInt(-self.pos_uncertainty, self.pos_uncertainty)
    def getANGLE(self): return self.angle # IN DEGREE

    def getBottomOffset(self): return 0
    def getAngleRelativeToVertical(self): return self.angle if self.angle < 180 else self.angle - 360

    def reset(self):
        self.angle = 0
        self.angle_speed = 0
        self.position = v.Vector(cf.window_x_size/2, cf.window_y_size/2)
        self.speed = v.NullVector()
        self.energy = 100

    def try_collect(self, collectable: collect.Collectable):
        if not collectable.isCollected() and collectable.isColliding(self.position):
            collectable.setCollected()
            self.manage_collected(collectable)
            print(collectable.type_name + " collected !")

    def manage_collected(self, collectable: collect.Collectable): return

class Drone(Machine):
    def __init__(self):
        super().__init__()
        self.name = "Drone n°" + str(self.id)

        self.angle_speed = 0
        self.speed = v.NullVector()

        self.l_motor = 0
        self.r_motor = 0

        self._l_motor, self._r_motor = [], []
        self._x_log, self._y_log = [], []
        self._speedx_log, self._speedy_log = [], []

        self.mass = 1 # Kg
        self.k_motor = 0.25
        self.k_angle = 500

        self.visual = pg.transform.scale(visuals.drone, self.size())

        self.pid_angle_max = 40
        self.pid_speedX_objective, self.pid_speedX_max = 0, 100
        self.pid_speedY_objective, self.pid_speedY_max = 0, 100
        self._pid_integral_X, self._pid_integral_X_max = 0, 5 * 1000 * cf.window_x_size
        self._pid_integral_Y, self._pid_integral_Y_max = 0, 1000 * cf.window_y_size

    def log(self):
        if len(self._l_motor) > 5 * cf.fps:
            self._l_motor = self._l_motor[1:]
        if len(self._r_motor) > 5 * cf.fps:
            self._r_motor = self._r_motor[1:]
        if len(self._x_log) > 5 * cf.fps:
            self._x_log = self._x_log[1:]
        if len(self._y_log) > 5 * cf.fps:
            self._y_log = self._y_log[1:]
        if len(self._speedx_log) > 5 * cf.fps:
            self._speedx_log = self._speedx_log[1:]
        if len(self._speedy_log) > 5 * cf.fps:
            self._speedy_log = self._speedy_log[1:]
        self._l_motor.append(self.l_motor)
        self._r_motor.append(self.r_motor)
        self._x_log.append(self.getXposition())
        self._y_log.append(self.getYposition())
        self._speedx_log.append(self.getXspeed())
        self._speedy_log.append(self.getYspeed())

    def draw(self):
        visual, blit_pos = u.RotateIMG(self.visual, self.angle, self.position(), (0.5 * self.size)())
        vr.window.blit(visual, blit_pos)

        pg.draw.line(vr.window, "black", (0, cf.window_y_size - lvls.getGroundHeight()),
                     (cf.window_x_size, cf.window_y_size - lvls.getGroundHeight()), 6)

        bar_size = 170
        pg.draw.line(vr.window, "white", (20, cf.window_y_size - 10), (20, cf.window_y_size - 10 - bar_size), 15)
        pg.draw.line(vr.window, "black", (20, cf.window_y_size - 10), (20, cf.window_y_size - 10 - bar_size * (self.l_motor/100)), 15)
        u.Text(str(int(self.l_motor)) + "%", (10, cf.window_y_size - 25 - bar_size), 12, "black")
        pg.draw.line(vr.window, "white", (cf.window_x_size - 20, cf.window_y_size - 10), (cf.window_x_size - 20, cf.window_y_size - 10 - bar_size), 15)
        pg.draw.line(vr.window, "black", (cf.window_x_size - 20, cf.window_y_size - 10),(cf.window_x_size - 20, cf.window_y_size - 10 - bar_size * (self.r_motor / 100)), 15)
        u.Text(str(int(self.r_motor)) + "%", (cf.window_x_size - 30, cf.window_y_size - 25 - bar_size), 12, "black")

        pg.draw.line(vr.window, "white", (20, cf.window_y_size - 220),(20, cf.window_y_size - 220 - bar_size*0.5), 15)
        pg.draw.line(vr.window, "red", (20, cf.window_y_size - 220),(20, cf.window_y_size - 220 - bar_size * 0.5 * (self.energy / 100)), 15)
        u.Text(str(int(self.energy)) + "%" if self.energy > 0 else "out of fuel !", (9, cf.window_y_size - 235 - bar_size*0.5), 12, "black")

        if cf.debug:
            u.Text(str(self) + " | " + str((round(self.position.x, 1), round(self.position.y, 1))) + " | " + str(round(self.angle, 1)) + " | " + str((round(self.speed.x, 1), round(self.speed.y, 1))) + " | " + str(round(self.angle_speed, 1)), (10, 10),12, 'black')

        graph_size = bar_size
        l_rect = pg.rect.RectType(35, cf.window_y_size - graph_size - 10, graph_size, graph_size)
        r_rect = pg.rect.RectType(cf.window_x_size - graph_size - 35, cf.window_y_size - graph_size - 10, graph_size, graph_size)
        m_rect = pg.rect.RectType(cf.window_x_size/2 - graph_size/2, cf.window_y_size - graph_size - 10, graph_size, graph_size)

        x_rect = pg.rect.RectType(35 + graph_size + 20, cf.window_y_size - graph_size - 10, graph_size, graph_size)
        y_rect = pg.rect.RectType(cf.window_x_size/2 - graph_size/2 + graph_size + 20, cf.window_y_size - graph_size - 10, graph_size, graph_size)

        l_points,r_points,m_points = [], [], []
        x_points, y_points = [], []
        for i, power in enumerate(self._l_motor):
            x = l_rect.right - (i * l_rect.width // len(self._l_motor))
            y = l_rect.bottom - 5 - (power * (l_rect.height - 10) // 100)  # On normalise entre 0 et 100
            l_points.append((int(x), int(y)))
        for i, power in enumerate(self._r_motor):
            x = r_rect.left + (i * r_rect.width // len(self._l_motor))
            y = r_rect.bottom - 5 - (power * (r_rect.height - 10) // 100)  # On normalise entre 0 et 100
            r_points.append((int(x), int(y)))
        for i, power in enumerate(self._l_motor):
            y = m_rect.top + (i * m_rect.width // len(self._l_motor))
            x = m_rect.left + m_rect.width/2 + ((self._l_motor[i] - self._r_motor[i]) * 0.5 * (m_rect.width - 5) // 100)  # On normalise entre 0 et 100
            m_points.append((int(x), int(y)))

        for i, power in enumerate(self._x_log):
            x = x_rect.right - (i * x_rect.width // len(self._x_log))
            y = x_rect.top + 5 + (power * (x_rect.height - 10) // cf.window_x_size)  # On normalise entre 0 et 100
            x_points.append((x, y))
        for i, power in enumerate(self._y_log):
            x = y_rect.right - (i * y_rect.width // len(self._x_log))
            y = y_rect.top + 5 + (power * (y_rect.height - 10) // cf.window_y_size)  # On normalise entre 0 et 100
            y_points.append((x, y))

        if len(l_points) > 1: pg.draw.lines(vr.window, "black", False, l_points, 1)
        pg.draw.rect(vr.window, "white", l_rect, 2)
        if len(r_points) > 1: pg.draw.lines(vr.window, "black", False, r_points, 1)
        pg.draw.rect(vr.window, "white", r_rect, 2)
        if len(m_points) > 1: pg.draw.lines(vr.window, "black", False, m_points, 1)
        pg.draw.rect(vr.window, "white", m_rect, 2)

        if len(x_points) > 1: pg.draw.lines(vr.window, "black", False, x_points, 1)
        pg.draw.rect(vr.window, "white", x_rect, 2)
        if len(y_points) > 1: pg.draw.lines(vr.window, "black", False, y_points, 1)
        pg.draw.rect(vr.window, "white", y_rect, 2)

    def reset(self):
        self.l_motor, self.r_motor = 0, 0
        self.energy = 100.
        self.speed = v.NullVector()
        self._pid_integral_X, self._pid_integral_Y = 0, 0
        super().reset()
        self.position = v.Vector(cf.window_x_size / 2, cf.window_y_size - lvls.getGroundHeight())

    def getBottomOffset(self):
        return self.size.x / 6
    def getHeightFromGround(self, exact=True): return cf.window_y_size - lvls.getGroundHeight() - self.getBottomOffset() - (self.position.y if exact else self.getYposition())

    def apply_physics(self):
        # Air friction
        self.speed.x = self.speed.x * 0.95
        self.speed.y = self.speed.y * 0.95
        self.angle_speed = self.angle_speed * 0.98

        # Gravity, perturbation...
        self.apply_external_forces()

        # Motors
        if self.energy > 0. :
            self.speed.y += -1 * vr.dt * t.cos(t.radians(self.angle)) * ((self.l_motor + self.r_motor) * self.k_motor) / self.mass
            self.speed.x += vr.dt * t.sin(t.radians(self.angle)) * ((self.l_motor + self.r_motor) * self.k_motor) / self.mass
            self.angle_speed += vr.dt * (self.l_motor - self.r_motor) * self.k_motor * self.k_angle / (self.mass * (self.size.x ** 2))

        # Energy lost
        self.energy = max(0., min(100., self.energy - vr.energy_loss * (abs(self.l_motor) + abs(self.r_motor))))

        # Collision
        right_modifier = v.Vector(self.size.x / 3, 0).rotated(self.angle, radians=False)
        left_modifier = v.Vector(-self.size.x / 3, 0).rotated(self.angle, radians=False)
        up_modifier = v.Vector(self.size.x / 6, 0).rotated(self.angle - 90, radians=False)
        down_modifier = v.Vector(self.size.x / 6, 0).rotated(self.angle + 90, radians=False)
        right = self.position + right_modifier
        left = self.position + left_modifier
        up = self.position + up_modifier
        down = self.position + down_modifier

        delta_angle_r, delta_angle_l = 0, 0
        displacement_r, displacement_l = v.NullVector(), v.NullVector()

        for collider in vr.colliders:

            collide_check, collider_element = collider.collide(right)
            if collide_check:
                gap = collider_element.center - right
                overlap = collider_element.radius - gap.norm
                if overlap > displacement_r.norm:
                    displacement_r = gap.normalised() * overlap
                    position_target = right - displacement_r
                    delta_angle_r = -1 * ((right - self.position).angle - (position_target - self.position).angle)
                if self.speed.x > 0 : self.speed.x = 0

            collide_check, collider_element = collider.collide(left)
            if collide_check:
                gap = collider_element.center - left
                overlap = collider_element.radius - gap.norm
                if overlap > displacement_l.norm:
                    displacement_l = gap.normalised() * overlap
                    position_target = left + displacement_l
                    delta_angle_l = (left - self.position).angle - (position_target - self.position).angle
                if self.speed.x < 0: self.speed.x = 0

            for position in (up, down):
                collide_check, collider_element = collider.collide(position)
                if collide_check:
                    if position == up:
                        if self.speed.y < 0 : self.speed.y = 0
                    elif position == down:
                        if self.speed.y > 0 : self.speed.y = 0
        # Correction of collision
        self.angle += t.degrees(delta_angle_r + delta_angle_l)
        self.position = self.position - displacement_r - displacement_l

    def apply_external_forces(self):
        self.speed.y += vr.dt * 9.81  # Gravity
        # self.speed.y += max(vr.dt * 9.81, 5 * vr.dt * 9.81 * (cf.window_y_size - self.position.y)/cf.window_y_size) # Gravity (Increase with height)
        for perturbation in vr.perturbations:
            self.speed += perturbation() * 9.81 * vr.dt

    def setLeftPower(self, percentage):
        if not (0 <= percentage <= 100):
            print("ERROR motor power is expressed in percentage (0 -> 100) %")
            return
        else:
            self.l_motor = percentage
    def setRightPower(self, percentage):
        if not (0 <= percentage <= 100):
            print("ERROR motor power is expressed in percentage (0 -> 100) %")
            return
        else:
            self.r_motor = percentage
    def increaseLeftPower(self, percentage):
        self.setLeftPower(max(0, min(100, self.l_motor + percentage)))
    def increaseRightPower(self, percentage):
        self.setRightPower(max(0, min(100, self.r_motor + percentage)))
    def getLeftPower(self): return self.l_motor
    def getRightPower(self): return self.r_motor

    def builtin_PID(self, target=None, kx=None, ky=None, reset=False, delta_max=20, angle_max=None, print_corr=False, xspeed_damp=True):
        if kx is None:
            kx = {"global": 1., "p": 15., "d": -30, "i": 0.002}
        if ky is None:
            ky = {"global": 1., "p": 0.5, "d": -0.08, "i": 0.03}
        if target is None:
            target = self.position[:]
        if reset:
            self._pid_integral_X = 0
            self._pid_integral_Y = 0
        if angle_max is None:
            angle_max = self.pid_angle_max

        X_objective = target.x
        # X Stabilisation Correction
        k = kx["global"]
        k_a = kx["p"]
        k_d = kx["d"]
        k_ix = kx["i"]

        ecart_X = X_objective - self.getXposition()

        self._pid_integral_X = min(self._pid_integral_X_max, max(-self._pid_integral_X_max, self._pid_integral_X + ecart_X * (20 if t.s(ecart_X) != t.s(self._pid_integral_X) else 1)))
        X_integral_factor = k_ix * self._pid_integral_X

        damp = 1
        if xspeed_damp:
            exp = round(max(0., min(0.3, 1. - 100. * abs(self._pid_integral_X / self._pid_integral_X_max))), 2)
            damp = min(1, (abs(ecart_X)/(10*self.radius))**exp)

        speedX_objective = damp * ((ecart_X + X_integral_factor) / cf.window_x_size) * self.pid_speedX_max
        speedX_objective = min(self.pid_speedX_max, max(-1 * self.pid_speedX_max, speedX_objective))  # Saturation en +- speedX_max

        angle_target = min(angle_max, max(-1 * angle_max, (speedX_objective/100) * angle_max))
        angle_proportional_integral_factor = k_a * u.angle_diff(angle_target, self.angle, rad=False)

        angle_speed_factor = k_d * self.getANGLEspeed()

        raw_correction = k * (angle_proportional_integral_factor + angle_speed_factor)
        correction = min(1, max(-1, raw_correction))
        if print_corr: print(f"X correction : raw = {raw_correction} | saturated = {correction}")
        delta = min(delta_max, max(abs(self.getRightPower() - self.getLeftPower()), 0.01), max(self.getRightPower(), self.getLeftPower(), 100 - self.getRightPower(), 100 - self.getLeftPower()))  # To keep the same total power
        self.increaseLeftPower(delta * correction)
        self.increaseRightPower(-1 * delta * correction)

        Y_objective = target.y
        # Y Stabilisation Correction
        speedY_objective = ((Y_objective - self.getYposition()) / cf.window_y_size) * self.pid_speedY_max
        speedY_objective = min(self.pid_speedY_max, max(-1 * self.pid_speedY_max, speedY_objective))  # Saturation en +- speedY_max

        k = ky["global"]
        k_a = ky["p"]
        k_d = ky["d"]
        k_i = ky["i"]

        Y_proportional_factor = k_a * (self.getYspeed() - speedY_objective)
        self._pid_integral_Y = min(self._pid_integral_Y_max, max(-self._pid_integral_Y_max, self._pid_integral_Y + Y_proportional_factor))
        Y_integral_factor = k_i * self._pid_integral_Y
        Y_speed_factor = k_d * abs(self.getRightPower() + self.getLeftPower()) * t.cos(t.radians(self.getANGLE()))
        raw_correction = k * (Y_proportional_factor + Y_speed_factor + Y_integral_factor)
        correction = min(1, max(-1, raw_correction))
        if print_corr: print(f"Y correction : raw = {raw_correction} | saturated = {correction}")

        delta = delta_max  # power of increase
        self.increaseLeftPower(delta * correction)
        self.increaseRightPower(delta * correction)

        return

    def manage_collected(self, collectable: collect.Collectable):
        if isinstance(collectable, collect.Energy):
            self.energy = 100
        elif isinstance(collectable, collect.Orb):
            vr.score += 1
        else:
            pass

class Car(Machine):
    def __init__(self):
        super().__init__()
        self.name = "Car n°" + str(self.id)

        self.engine_power = 0 # %
        self.steering = 0 # °
        self._max_steer = 45 # °

        self.mass = 1  # Kg
        self.k_motor = 200 # engine_power * k_motor -> acceleration
        self.k_steering = 0.02 # direction * k_angle -> angular acceleration
        self._top_speed_approx = self.k_motor * 0.048  # pix/s

        self.visual = pg.transform.scale(visuals.car, self.size())

    def reset(self):
        self.engine_power = 0  # %
        self.steering = 0  # °
        super().reset()

    def getEnginePower(self): return self.engine_power
    def setEnginePower(self, value): self.engine_power = min(100., max(-100, value))
    def increaseEnginePower(self, value): self.setEnginePower(self.getEnginePower() + value)

    def getSteering(self): return self.steering
    def setSteering(self, value): self.steering = self._max_steer * 0.01 * min(100., max(-100 , value))
    def increaseSteering(self, value): self.setSteering(100 * self.getSteering()/self._max_steer + value)

    def manage_collected(self, collectable: collect.Collectable):
        if isinstance(collectable, collect.Energy):
            self.energy = 100
        elif isinstance(collectable, collect.Orb):
            vr.score += 1
        else:
            pass

    def draw(self):
        visual, blit_pos = u.RotateIMG(self.visual, self.angle, self.position() + v.Vector(0.,0.), (0.5 * self.size.x, 0.5 * self.size.y))
        vr.window.blit(visual, blit_pos)

        bar_size = 170
        pg.draw.line(vr.window, "white", (20, cf.window_y_size - 10), (20, cf.window_y_size - 10 - bar_size), 15)
        pg.draw.line(vr.window, "black", (20, cf.window_y_size - 10 - bar_size*0.5), (20, cf.window_y_size - 10 - bar_size * ((100 + self.engine_power) / 200)), 15)
        u.Text(str(int(self.engine_power)) + "%", (10, cf.window_y_size - 25 - bar_size), 12, "black")

        pg.draw.line(vr.window, "white", (20 + 20, cf.window_y_size - 10), (20 + 20, cf.window_y_size - 10 - bar_size), 15)
        pg.draw.line(vr.window, "black", (20 + 20, cf.window_y_size - 10 - bar_size*0.5),(20 + 20, cf.window_y_size - 10 - bar_size * (self._max_steer + self.steering) / (2 * self._max_steer)), 15)
        u.Text(str(int(self.steering)) + "%", (10 + 20, cf.window_y_size - 25 - bar_size), 12, "black")

        pg.draw.line(vr.window, "white", (20, cf.window_y_size - 220), (20, cf.window_y_size - 220 - bar_size * 0.5),
                     15)
        pg.draw.line(vr.window, "red", (20, cf.window_y_size - 220),
                     (20, cf.window_y_size - 220 - bar_size * 0.5 * (self.energy / 100)), 15)
        u.Text(str(int(self.energy)) + "%" if self.energy > 0 else "out of fuel !",
               (9, cf.window_y_size - 235 - bar_size * 0.5), 12, "black")

    def apply_physics(self):
        # Air friction
        self.speed = 0.9 * self.speed
        self.angle_speed = 0.8 * self.angle_speed
        # Gravity, perturbation...
        self.apply_external_forces()

        # Motors
        pointing_vector = v.Vector(t.sin(t.radians(self.angle)), -t.cos(t.radians(self.angle)))
        right_vector = pointing_vector.rotated(90, radians=False)
        relative_speed = self.speed.dot(pointing_vector)
        if self.energy > 0. :
            angle_speed_factor = (relative_speed - 0.01 * self._top_speed_approx) * (0.2 + 1. - abs(relative_speed)/self._top_speed_approx)
            angle_acceleration = self.k_steering * self.steering * angle_speed_factor / self.mass
            v_acceleration = self.k_motor * self.engine_power * pointing_vector / (self.mass * (self.size.x ** 2))
            self.speed += vr.dt * v_acceleration
            self.angle_speed += vr.dt * angle_acceleration
        # Energy lost
        self.energy = max(0., min(100., self.energy - vr.energy_loss * abs(self.engine_power)))

        # Collision
        _right_modifier = v.Vector(self.size.x * 0.25, 0).rotated(self.angle, radians=False)
        _left_modifier = v.Vector(self.size.x * 0.25, 0).rotated(self.angle + 180, radians=False)
        _up_modifier = v.Vector(self.size.y * 0.35, 0).rotated(self.angle - 90, radians=False)
        _down_modifier = v.Vector(self.size.y * 0.35, 0).rotated(self.angle + 90, radians=False)
        right = self.position + _right_modifier
        left = self.position + _left_modifier
        up = self.position + _up_modifier
        down = self.position + _down_modifier

        displacement_correction = v.NullVector()
        for collider in vr.colliders:
            for position in (up, down, left, right):
                collide_check, collider_element = collider.collide(position)
                if collide_check:
                    gap = collider_element.center - position
                    overlap = max(0, collider_element.radius - gap.norm)
                    displacement_correction += gap.normalised() * overlap
                    self.speed = self.speed - max(0, self.speed.dot(gap.normalised())) * gap.normalised()
        # Correction of collision
        self.position = self.position - displacement_correction

    def apply_external_forces(self):
        for perturbation in vr.perturbations:
            self.speed += perturbation() * 9.81 * vr.dt

