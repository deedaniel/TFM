from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError

DIR_PATH = dirname(abspath(__file__))
TTT_FILE = 'pick_and_place.ttt'


# Definicion de la estructura del robot
class Robot(object):
    def __init__(self, robot_arm, robot_gripper, robot_tip):
        self.arm = robot_arm
        self.gripper = robot_gripper
        self.tip = robot_tip
        self.pos = robot_arm.get_position()


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self):
        self.block = Shape('block')
        self.pick = Shape('large_container')
        self.place = Shape('small_container')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.wp2 = Dummy('waypoint2')
        self.wp3 = Dummy('waypoint3')
        self.pick_wp = Dummy('pick')
        self.place_wp = Dummy('place')
        self.success = ProximitySensor('success')


class Parameters(object):
    def __init__(self):
        self.time = 0
        self.iteration = 1


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []
        self.iterations = []


class PickAndPlace(object):
    def __init__(self, headless_mode: bool):
        self.pr = PyRep()
        self.pr.launch(join(DIR_PATH, TTT_FILE), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask()
        self.param = Parameters()
        self.lists = Lists()

    def pick_and_place(self, wp_params: np.array):
        # Definición de los waypoint de grasp y dejada
        pick_pos_rel = np.array([wp_params[0], wp_params[1], wp_params[2]])
        place_pos_rel = np.array([wp_params[3], wp_params[4], wp_params[5]])

        pick_pos = self.task.wp0.get_position() + pick_pos_rel
        place_pos = self.task.wp2.get_position() + place_pos_rel

        self.task.wp1.set_position(pick_pos)
        self.task.wp3.set_position(place_pos)
        print(self.task.wp3.get_position())

        tray = [self.task.wp0, self.task.wp1, self.task.wp0, self.task.wp2, self.task.wp3, self.task.wp2]

        # Ejecución de la trayectoria
        self.pr.start()
        reward = 0

        for pos in tray:
            try:
                path = self.robot.arm.get_path(position=pos.get_position(),
                                               euler=pos.get_orientation())
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pr.step()

                if pos == self.task.wp1:
                    done = False
                    # Open the gripper halfway at a velocity of 0.04.
                    while not done:
                        done = self.robot.gripper.actuate(0, velocity=0.04)
                        self.pr.step()
                    self.robot.gripper.grasp(self.task.block)
                    distance_pick = calc_distance(self.robot.tip.get_position(), self.task.pick_wp.get_position())
                    reward -= 400 * distance_pick ** 2
                elif pos == self.task.wp3:
                    done = False
                    # Open the gripper halfway at a velocity of 0.04.
                    while not done:
                        done = self.robot.gripper.actuate(1, velocity=0.04)
                        self.pr.step()
                    self.robot.gripper.release()
                    distance_place = calc_distance(self.task.block.get_position(), self.task.place_wp.get_position())
                    reward -= 400 * distance_place ** 2
            except ConfigurationPathError as e:
                reward = -85
                print('Could not find path')

        self.pr.stop()  # Stop the simulation
        self.lists.list_of_parameters = np.append(self.lists.list_of_parameters, wp_params)
        self.lists.list_of_rewards = np.append(self.lists.list_of_rewards, reward)
        self.lists.iterations = np.append(self.lists.iterations, self.param.iteration)
        self.param.iteration += 1
        return -reward

    def shutdown(self):
        self.pr.shutdown()  # Close the application

    def clean_lists(self):
        self.lists = Lists()

    def return_lists(self):
        return self.lists


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
