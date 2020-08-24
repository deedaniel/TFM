from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

DIR_PATH = dirname(abspath(__file__))
TTT_FILE = 'push_button.ttt'


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self):
        self.target_button = Shape('push_button_target')
        self.target_topPlate = Shape('target_button_topPlate')
        self.target_wrap = Shape('target_button_wrap')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.joint = Joint('target_button_joint')


class Parameters(object):
    def __init__(self):
        self.time = 0
        self.iteration = 1
        self.original_pos = 0


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []
        self.iterations = []


class PushButton(object):
    def __init__(self):
        self.pyrep = PyRep()
        self.pyrep.launch(join(DIR_PATH, TTT_FILE), headless=False)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask()
        self.param = Parameters()
        self.param.original_pos = self.task.joint.get_joint_position()
        self.lists = Lists()

    def push_button(self, push_params: np.array):
        # Definición del punto de empuje
        push_pos_rel = np.array([push_params[0], push_params[1], push_params[2]])
        push_pos = self.task.target_button.get_position() + push_pos_rel
        self.task.wp1.set_position(push_pos)

        tray = [self.task.wp0, self.task.wp1]

        # Ejecución de la trayectoria
        self.pyrep.start()
        self.param.time = 0
        reward = 0

        done = False
        # Cerrar la pinza para poder pretar el boton.
        while not done:
            done = self.robot.gripper.actuate(0, velocity=0.05)
            self.pyrep.step()

        for pos in tray:
            try:
                path = self.robot.arm.get_path(position=pos.get_position(),
                                               euler=pos.get_orientation())
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pyrep.step()
                reward = 2000 * np.abs(np.linalg.norm(self.task.joint.get_joint_position()- self.param.original_pos)
                                       - 0.003)
            except ConfigurationPathError:
                print('Could not find path')
                reward = -40
                return -reward

        self.pyrep.stop()  # Stop the simulation
        self.lists.list_of_rewards = np.append(self.lists.list_of_rewards, -reward)
        self.lists.list_of_parameters = np.append(self.lists.list_of_parameters, push_params)
        self.lists.iterations = np.append(self.lists.iterations, self.param.iteration)
        self.param.iteration += 1
        return -reward

    def clean_lists(self):
        self.lists = Lists()

    def return_lists(self):
        return self.lists

    def shutdown(self):
        self.pyrep.shutdown()  # Close the application


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
