from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
import pickle
import sys

DIR_PATH = dirname(abspath(__file__))
TTT_FILE = 'avoid_obstacle.ttt'


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.link5 = Shape("Panda_link5_visual")
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


class Target(object):  # Estructura del objetivo
    def __init__(self):
        self.dummy = Dummy('target0')
        self.initial_pos = self.dummy.get_position()
        self.pos = []


class Waypoints(object):
    def __init__(self):
        self.initial_pos = Dummy('target0')
        self.final_pos = Dummy('target1')


class Obstacle(object):  # Estructura del obstaculo
    def __init__(self):
        self.obstacle = Shape('obstacle')
        self.pos = self.obstacle.get_position()  # Posicion del obstaculo
        self.radius = 0.3  # Radio del obstaculo


class Parameters(object):
    def __init__(self):
        self.linear_delta = 0.05  # 5 centimetros
        # Parametros de evitacion: radio y numero de pasos para hacer la trayectoria
        self.radius = 0.3
        self.steps = 10
        self.circular_delta = np.pi / (self.steps - 1)  # 180 / (steps-1) grados para hacer semicircunferencia
        self.time = 0
        self.coords = 'cartesianas'


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []


class AvoidObstacle(object):
    def __init__(self, headless_mode: bool):
        self.pyrep = PyRep()
        self.pyrep.launch(join(DIR_PATH, TTT_FILE), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.obstacle = Obstacle()
        self.target = Target()
        self.param = Parameters()
        self.waypoints = Waypoints()
        self.lists = Lists()

    def tray_circular(self, radius):
        self.pyrep.start()  # We start the simulation

        self.target.pos = self.target.initial_pos  # We set the pos as the initial pos

        # We try to get a path to the initial target of the path
        try:
            path = self.robot.arm.get_path(position=self.target.pos,
                                           euler=[0, np.radians(180), 0])

            # We execute the path
            done = False
            while not done:
                done = path.step()
                self.pyrep.step()
            print('Reached initial target.')
        except ConfigurationPathError:
            print('Could not find path.')
            exit()

        distance_3d = np.array(self.obstacle.pos - self.target.pos)
        distance = np.linalg.norm(distance_3d)

        # Mienstras la distancia al objecto sea menor que el radio del obstaculo, hacemos trayctoria lineal
        while (distance - self.param.linear_delta) > radius:
            # Calcular la siguiente posición
            next_pos = self.target.pos + np.array([0, self.param.linear_delta, 0])
            self.target.pos = next_pos

            try:
                path = self.robot.arm.get_path(position=self.target.pos,
                                               euler=[0, np.radians(180), 0],
                                               ignore_collisions=True)
            except ConfigurationPathError:
                print('Could not find path')
                continue

            # Step the simulation and advance the agent along the path
            done = False
            while not done:
                done = path.step()
                self.pyrep.step()

                distance = calc_distance(self.obstacle.pos, self.target.pos)

        self.param.time = 0  # Inicializamos el tiempo

        # Hacemos trayectoria circular
        for step in range(self.param.steps):
            # Calculate the next target
            next_pos = self.obstacle.pos + np.array([0,
                                                     -radius * np.cos(step * self.param.circular_delta),
                                                     radius * np.sin(step * self.param.circular_delta)])
            self.target.pos = next_pos

            try:
                path = self.robot.arm.get_path(position=self.target.pos,
                                               euler=[0, np.radians(180), 0],
                                               ignore_collisions=True)
            except ConfigurationPathError:
                print('Could not find path')
                continue

            # Step the simulation and advance the agent along the path
            done = False
            while not done:
                done = path.step()
                self.pyrep.step()
                self.param.time += self.pyrep.get_simulation_timestep()

        reward = (-(20 * (1 - self.obstacle.radius / radius)) ** 2 + 2) + 10 / self.param.time
        self.pyrep.stop()  # Stop the simulation
        self.lists.list_of_parameters = np.append(self.lists.list_of_parameters, radius)
        self.lists.list_of_rewards = np.append(self.lists.list_of_rewards, -reward)
        return -reward

    def avoidance_brute_force(self, radius_interval: list):
        radius_step = 0.005
        self.clean_lists()
        self.param.radius = radius_interval[0]

        while self.param.radius <= radius_interval[1]:
            self.tray_circular(self.param.radius)
            self.param.radius += radius_step

        pickle.dump(self.lists, open("listas_brute_force.p", "wb"))

    def tray_with_waypoints(self, wp_params: np.array):
        print(wp_params)
        if self.param.coords == 'cartesianas':
            waypoint1, waypoint2 = self.get_waypoints_cart(wp_params)
        elif self.param.coords == 'esfericas':
            waypoint1, waypoint2 = self.get_waypoints_esf(wp_params)
        else:
            waypoint1, waypoint2 = [], []
            print('Error al definir el tipo de coordenadas')
            sys.exit()

        # Definición de la trayectoria
        tray = [self.waypoints.initial_pos, waypoint1, waypoint2, self.waypoints.final_pos]

        d_tray_1 = self.waypoints.initial_pos.check_distance(waypoint1)
        d_tray_2 = waypoint1.check_distance(waypoint2)
        d_tray_3 = waypoint2.check_distance(self.waypoints.final_pos)
        d_tray = d_tray_1 + d_tray_2 + d_tray_3

        r_long = - 4 * d_tray ** 2
        r_obstacle = 0.0

        # Ejecución de la trayectoria
        self.pyrep.start()

        for pos in tray:
            try:
                path = self.robot.arm.get_linear_path(position=pos.get_position(),
                                                      euler=[0, np.radians(180), 0])
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pyrep.step()

                    distance_obstacle_gripper = self.robot.gripper.check_distance(self.obstacle.obstacle)
                    distance_obstacle_robot = self.robot.link5.check_distance(self.obstacle.obstacle)
                    r_obstacle -= (20 * np.exp(-150 * distance_obstacle_gripper) +
                                   20 * np.exp(-150 * distance_obstacle_robot))
            except ConfigurationPathError:
                print('Could not find path')
                reward = -300
                print(reward)
                self.pyrep.stop()
                self.lists.list_of_parameters.append(list(wp_params))
                self.lists.list_of_rewards.append(reward)
                return -reward

        reward = r_long + r_obstacle
        print(reward)

        self.pyrep.stop()
        self.lists.list_of_parameters.append(list(wp_params))
        self.lists.list_of_rewards.append(reward)
        return -reward

    def shutdown(self):
        self.pyrep.shutdown()  # Close the application

    def clean_lists(self):
        self.lists = Lists()

    def return_lists(self):
        return self.lists

    def set_coords(self, coords: str):
        self.param.coords = coords

    def get_coords(self):
        return self.param.coords

    def get_waypoints_cart(self, wp_params: np.array):
        pos1_rel = np.array([wp_params[0], wp_params[1], wp_params[2]])
        pos1_abs = pos1_rel + self.waypoints.initial_pos.get_position()
        waypoint1 = Dummy.create()
        waypoint1.set_position(pos1_abs)

        pos2_rel = np.array([wp_params[3], wp_params[4], wp_params[5]])
        pos2_abs = pos2_rel + pos1_abs
        waypoint2 = Dummy.create()
        waypoint2.set_position(pos2_abs)

        return waypoint1, waypoint2

    def get_waypoints_esf(self, wp_params: np.array):
        radio1 = wp_params[0]
        tita1 = wp_params[1]
        phi1 = wp_params[2]
        pos1_rel = np.array([radio1*np.sin(tita1)*np.cos(phi1),
                             radio1*np.sin(tita1)*np.sin(phi1),
                             radio1*np.cos(tita1)])
        pos1_abs = pos1_rel + self.waypoints.initial_pos.get_position()
        waypoint1 = Dummy.create()
        waypoint1.set_position(pos1_abs)

        radio2 = wp_params[3]
        tita2 = wp_params[4]
        phi2 = wp_params[5]
        pos2_rel = np.array([radio2*np.sin(tita2)*np.cos(phi2),
                             radio2*np.sin(tita2)*np.sin(phi2),
                             radio2*np.cos(tita2)])
        pos2_abs = pos2_rel + pos1_abs
        waypoint2 = Dummy.create()
        waypoint2.set_position(pos2_abs)

        return waypoint1, waypoint2


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
