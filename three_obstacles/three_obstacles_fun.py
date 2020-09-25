from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

DIR_PATH = dirname(abspath(__file__))
TTT_FILE = 'three_obstacles.ttt'


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


class InitTask(object):  # Estructura del obstaculo
    def __init__(self):
        self.initial_pos = Dummy('target0')
        self.final_pos = Dummy('target1')
        self.obstacle0 = Shape('Cylinder')
        self.obstacle1 = Shape('Cylinder0')
        self.obstacle2 = Shape('Cylinder1')
        self.sensor = ProximitySensor('Panda_sensor')


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []


class ThreeObstacles(object):
    def __init__(self, headless_mode: bool):
        self.pyrep = PyRep()
        self.pyrep.launch(join(DIR_PATH, TTT_FILE), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask()
        self.lists = Lists()

    def avoidance_with_waypoints(self, wp_params: np.array):
        waypoint1, waypoint2 = self.get_waypoints_esf(wp_params)

        # Definición de la trayectoria
        tray = [self.task.initial_pos, waypoint1, waypoint2, self.task.final_pos]

        d_tray_1 = self.task.initial_pos.check_distance(waypoint1)
        d_tray_2 = waypoint1.check_distance(waypoint2)
        d_tray_3 = waypoint2.check_distance(self.task.final_pos)
        d_tray = d_tray_1 + d_tray_2 + d_tray_3

        # Ejecución de la trayectoria
        self.pyrep.start()
        cost = 2 * d_tray ** 2

        for pos in tray:
            try:
                path = self.robot.arm.get_path(position=pos.get_position(),
                                               euler=[0, np.radians(180), 0],
                                               ignore_collisions=True)
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pyrep.step()

                    distance_obstacle0 = self.robot.gripper.check_distance(self.task.obstacle0)
                    distance_obstacle1 = self.robot.gripper.check_distance(self.task.obstacle1)
                    distance_obstacle2 = self.robot.gripper.check_distance(self.task.obstacle2)

                    cost += (20 * np.exp(-300 * distance_obstacle0) +
                             20 * np.exp(-300 * distance_obstacle1) +
                             20 * np.exp(-300 * distance_obstacle2))
            except ConfigurationPathError:
                cost = 400

        self.pyrep.stop()
        self.lists.list_of_parameters.append(list(wp_params))
        self.lists.list_of_rewards.append(cost)
        return cost

    def shutdown(self):
        self.pyrep.shutdown()  # Close the application

    def clean_lists(self):
        self.lists = Lists()

    def return_lists(self):
        return self.lists

    def get_waypoints_esf(self, wp_params: np.array):
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
