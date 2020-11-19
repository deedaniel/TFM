from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError

DIR_PATH = dirname(abspath(__file__))


# Definicion de la estructura del robot
class Robot(object):
    def __init__(self, robot_arm, robot_gripper, robot_tip):
        self.arm = robot_arm
        self.gripper = robot_gripper
        self.tip = robot_tip
        self.pos = robot_arm.get_position()


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self, variation: str):
        self.block = Shape('block')
        self.init_pos = self.block.get_position()
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.wp2 = Dummy('waypoint2')
        self.wp3 = Dummy('waypoint3')
        self.pick_wp = Dummy('pick')
        self.place_wp0 = Dummy('place0')
        if variation == '2container':
            self.place_wp1 = Dummy('place1')


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []
        self.iterations = []


class PickAndPlace(object):
    def __init__(self, headless_mode: bool, variation="1container"):
        self.pr = PyRep()
        self.variation = variation
        self.ttt_file = 'pick_and_place_' + self.variation + '.ttt'
        self.pr.launch(join(DIR_PATH, self.ttt_file), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask(self.variation)
        self.lists = Lists()

    def pick_and_place(self, wp_params: np.array):
        print(wp_params)
        # Definición de los waypoint de grasp y dejada
        pick_pos_rel = np.array([wp_params[0], wp_params[1], wp_params[2]])
        place_pos_rel = np.array([wp_params[3], wp_params[4], wp_params[5]])

        pick_pos = self.task.wp0.get_position() + pick_pos_rel
        place_pos = self.task.wp2.get_position() + place_pos_rel

        self.task.wp1.set_position(pick_pos)
        self.task.wp3.set_position(place_pos)

        tray = [self.task.wp0, self.task.wp1, self.task.wp0, self.task.wp2, self.task.wp3, self.task.wp2]

        # Ejecución de la trayectoria
        self.pr.start()
        self.task.block.set_position(self.task.init_pos)

        distance_pick = 0.0
        distance_tip0 = 0.0
        distance_tip1 = 0.0
        distance_place1 = 0.0

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
                    distance_pick = self.robot.tip.check_distance(self.task.pick_wp)
                elif pos == self.task.wp3:
                    done = False
                    # Open the gripper halfway at a velocity of 0.04.
                    while not done:
                        done = self.robot.gripper.actuate(1, velocity=0.04)
                        self.pr.step()
                    self.robot.gripper.release()
                    distance_tip0 = self.robot.tip.check_distance(self.task.place_wp0)
                    if self.variation == '2container':
                        distance_tip1 = self.robot.tip.check_distance(self.task.place_wp1)
            except ConfigurationPathError:
                print('Could not find path')
                reward = -750

                self.pr.stop()  # Stop the simulation
                self.lists.list_of_parameters.append(wp_params)
                self.lists.list_of_rewards.append(reward)
                return -reward

        distance_place0 = calc_distance(self.task.block.get_position(), self.task.place_wp0.get_position())
        if self.variation == '2container':
            distance_place1 = calc_distance(self.task.block.get_position(), self.task.place_wp1.get_position())

        reward = - (200 * distance_pick ** 2 + 200 * distance_place0 ** 2 + 400 * distance_place1 ** 2
                    + 3500 * distance_place0 * distance_place1 + 100 * distance_tip0 ** 2 + 200 * distance_tip1 ** 2 +
                    + 1750 * distance_tip0 * distance_tip1)

        self.pr.stop()  # Stop the simulation
        self.lists.list_of_parameters.append(wp_params)
        self.lists.list_of_rewards.append(reward)
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
