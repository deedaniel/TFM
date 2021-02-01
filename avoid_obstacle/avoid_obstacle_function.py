from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
import sys

DIR_PATH = dirname(abspath(__file__))


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.link5 = Shape("Panda_link5_visual")
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


class InitTask(object):
    def __init__(self, variation: str):
        self.initial_pos = Dummy('target0')
        self.final_pos = Dummy('target1')
        if variation == '1obstacle':
            self.obstacle = Shape('obstacle')
        elif variation == '3obstacle':
            self.obstacle0 = Shape('Cylinder')
            self.obstacle1 = Shape('Cylinder0')
            self.obstacle2 = Shape('Cylinder1')


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []


class AvoidObstacle(object):
    def __init__(self, headless_mode: bool, variation='1obstacle', coords='cartesianas'):
        # Inicialización de la tarea, lanzando PyRep, cargando la escena en el simulador, cargando el robot y los
        # elementos de la escena y inicializando las listas.
        self.pyrep = PyRep()
        self.variation = variation
        self.coords = coords
        self.ttt_file = 'avoid_obstacle_' + self.variation + '.ttt'
        self.pyrep.launch(join(DIR_PATH, self.ttt_file), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask(self.variation)
        self.lists = Lists()

    def avoid1obstacle(self, wp_params: np.array):
        if self.coords == 'cartesianas':
            waypoint1, waypoint2 = self.get_waypoints_cart3d(wp_params)
        elif self.coords == 'esfericas':
            waypoint1, waypoint2 = self.get_waypoints_esf3d(wp_params)
        else:
            print('Error al definir el tipo de coordenadas')
            sys.exit()

        # Definición de la trayectoria
        tray = [self.task.initial_pos, waypoint1, waypoint2, self.task.final_pos]

        d_tray_1 = self.task.initial_pos.check_distance(waypoint1)
        d_tray_2 = waypoint1.check_distance(waypoint2)
        d_tray_3 = waypoint2.check_distance(self.task.final_pos)
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

                    distance_obstacle_gripper = self.robot.gripper.check_distance(self.task.obstacle)
                    distance_obstacle_robot = self.robot.link5.check_distance(self.task.obstacle)
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

    def avoid3obstacles(self, wp_params: np.array):
        if self.coords == 'cartesianas':
            waypoint1, waypoint2 = self.get_waypoints_cart2d(wp_params)
        elif self.coords == 'esfericas':
            waypoint1, waypoint2 = self.get_waypoints_esf2d(wp_params)
        else:
            print('Error al definir el tipo de coordenadas')
            sys.exit()

        # Definición de la trayectoria
        tray = [self.task.initial_pos, waypoint1, waypoint2, self.task.final_pos]

        d_tray_1 = self.task.initial_pos.check_distance(waypoint1)
        d_tray_2 = waypoint1.check_distance(waypoint2)
        d_tray_3 = waypoint2.check_distance(self.task.final_pos)
        d_tray = d_tray_1 + d_tray_2 + d_tray_3

        # Ejecución de la trayectoria
        self.pyrep.start()
        reward_long = - 4 * d_tray ** 2
        reward_dist = 0.0

        for pos in tray:
            try:
                path = self.robot.arm.get_linear_path(position=pos.get_position(),
                                                      euler=[0.0, np.radians(180), 0.0])
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pyrep.step()

                    distance_obstacle0 = self.robot.gripper.check_distance(self.task.obstacle0)
                    distance_obstacle1 = self.robot.gripper.check_distance(self.task.obstacle1)
                    distance_obstacle2 = self.robot.gripper.check_distance(self.task.obstacle2)

                    reward_dist -= (20 * np.exp(-300 * distance_obstacle0) +
                                    20 * np.exp(-300 * distance_obstacle1) +
                                    20 * np.exp(-300 * distance_obstacle2))
            except ConfigurationPathError:
                reward = -400.0
                self.pyrep.stop()
                self.lists.list_of_parameters.append(list(wp_params))
                self.lists.list_of_rewards.append(reward)
                return -reward

        reward = reward_long + reward_dist

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

    def get_waypoints_cart3d(self, wp_params: np.array):
        pos1_rel = np.array([wp_params[0], wp_params[1], wp_params[2]])
        pos1_abs = pos1_rel + self.task.initial_pos.get_position()
        waypoint1 = Dummy.create()
        waypoint1.set_position(pos1_abs)

        pos2_rel = np.array([wp_params[3], wp_params[4], wp_params[5]])
        pos2_abs = pos2_rel + pos1_abs
        waypoint2 = Dummy.create()
        waypoint2.set_position(pos2_abs)

        return waypoint1, waypoint2

    def get_waypoints_esf3d(self, wp_params: np.array):
        radio1 = wp_params[0]
        tita1 = wp_params[1]
        phi1 = wp_params[2]
        pos1_rel = np.array([radio1*np.sin(tita1)*np.cos(phi1),
                             radio1*np.sin(tita1)*np.sin(phi1),
                             radio1*np.cos(tita1)])
        pos1_abs = pos1_rel + self.task.initial_pos.get_position()
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

    def get_waypoints_cart2d(self, wp_params: np.array):
        pos1_rel = np.array([wp_params[0], wp_params[1], 0.0])
        pos1_abs = pos1_rel + self.task.initial_pos.get_position()
        waypoint1 = Dummy.create()
        waypoint1.set_position(pos1_abs)

        pos2_rel = np.array([wp_params[2], wp_params[3], 0.0])
        pos2_abs = pos2_rel + pos1_abs
        waypoint2 = Dummy.create()
        waypoint2.set_position(pos2_abs)

        return waypoint1, waypoint2

    def get_waypoints_esf2d(self, wp_params: np.array):
        radio1 = wp_params[0]
        tita1 = wp_params[1]
        pos1_rel = np.array([radio1 * np.sin(tita1),
                             radio1 * np.cos(tita1),
                             0])
        pos1_abs = pos1_rel + self.task.initial_pos.get_position()
        waypoint1 = Dummy.create()
        waypoint1.set_position(pos1_abs)

        radio2 = wp_params[2]
        tita2 = wp_params[3]
        pos2_rel = np.array([radio2 * np.sin(tita2),
                             radio2 * np.cos(tita2),
                             0])
        pos2_abs = pos2_rel + pos1_abs
        waypoint2 = Dummy.create()
        waypoint2.set_position(pos2_abs)

        return waypoint1, waypoint2


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
