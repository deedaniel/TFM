from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.errors import ConfigurationPathError
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

DIR_PATH = dirname(abspath(__file__))


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self, variation):
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.button_wp0 = Dummy('target_button0')
        self.joint0 = Joint('target_button_joint0')
        if variation == '2button':
            self.button_wp1 = Dummy('target_button1')
            self.joint1 = Joint('target_button_joint1')


class Parameters(object):
    def __init__(self, variation):
        self.original_pos0 = 0
        if variation == '2button':
            self.original_pos1 = 0


class Lists(object):
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []
        self.iterations = []


class PushButton(object):
    def __init__(self, headless_mode: bool, variation='1button'):
        self.pyrep = PyRep()
        self.variation = variation
        self.ttt_file = 'push_button_' + self.variation + '.ttt'
        self.pyrep.launch(join(DIR_PATH, self.ttt_file), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask(self.variation)
        self.param = Parameters(self.variation)
        self.param.original_pos0 = self.task.joint0.get_joint_position()
        if self.variation == '2button':
            self.param.original_pos1 = self.task.joint1.get_joint_position()
        self.lists = Lists()

    def push_button(self, push_params: np.array):
        # Definición del punto de empuje
        push_pos_rel = np.array([push_params[0], push_params[1], push_params[2]])
        push_pos = self.task.wp0.get_position() + push_pos_rel
        self.task.wp1.set_position(push_pos)

        # Definicion de la orientacion
        or_rel = np.array([push_params[3], push_params[4], push_params[5]])
        or_abs = self.task.wp0.get_orientation() + or_rel
        self.task.wp0.set_orientation(or_abs)
        self.task.wp1.set_orientation(or_abs)

        tray = [self.task.wp0, self.task.wp1]

        # Ejecución de la trayectoria
        self.pyrep.start()
        self.param.time = 0
        reward = 0

        done = False
        # Cerrar la pinza para poder apretar el boton.
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
                distance_objective0 = self.robot.tip.check_distance(self.task.button_wp0)
                error_alpha = self.task.button_wp0.get_orientation()[0] - self.task.wp1.get_orientation()[0]
                error_beta = self.task.button_wp0.get_orientation()[1] - self.task.wp1.get_orientation()[1]
                error_gamma = self.task.button_wp0.get_orientation()[2] - self.task.wp1.get_orientation()[2]
                reward = (-400 * distance_objective0 ** 2 - 5 * error_alpha ** 2 - 5 * error_beta ** 2
                          - 1 * error_gamma ** 2)
                if self.variation == '2button':
                    distance_objective1 = self.robot.tip.check_distance(self.task.button_wp1)
                    reward -= 800 * distance_objective1 ** 2
            except ConfigurationPathError:
                print('Could not find path')
                reward = -150
                return -reward

        self.pyrep.stop()  # Stop the simulation
        self.lists.list_of_rewards.append(reward)
        self.lists.list_of_parameters.append(list(push_params))
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
