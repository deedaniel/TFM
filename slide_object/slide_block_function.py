from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

DIR_PATH = dirname(abspath(__file__))
TTT_FILE = 'slide_block.ttt'


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self):
        self.block = Shape('block')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.target = Shape('target')
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


class SlideBlockFunction(object):
    def __init__(self):
        self.pyrep = PyRep()
        self.pyrep.launch(join(DIR_PATH, TTT_FILE), headless=True)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask()
        self.param = Parameters()
        self.lists = Lists()

    def slide_block(self, slide_params: np.array):
        # Definición del objetivo final
        distance = slide_params[0]
        orientation = slide_params[1]

        final_pos_rel = np.array([distance * math.sin(orientation), distance * math.cos(orientation), 0])
        final_pos_abs = final_pos_rel + self.task.wp0.get_position()

        final_wp = Dummy.create()
        final_wp.set_position(final_pos_abs)
        final_wp.set_orientation(self.task.wp0.get_orientation())

        tray = [self.task.wp0, final_wp]

        # Ejecución de la trayectoria
        self.pyrep.start()

        self.param.time = 0
        reward = 0

        done = False
        # Cerrar la pinza para poder empujar el objeto.
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
                    self.param.time += self.pyrep.get_simulation_timestep()
                    distance_objective = calc_distance(self.task.wp1.get_position(), self.task.block.get_position())
                    reward += math.exp(-distance_objective)
            except ConfigurationPathError:
                print('Could not find path')
                reward = 0
                return -reward

        self.pyrep.stop()  # Stop the simulation
        self.lists.list_of_rewards = np.append(self.lists.list_of_rewards, -reward)
        self.lists.list_of_parameters = np.append(self.lists.list_of_parameters, slide_params)
        self.lists.iterations = np.append(self.lists.iterations, self.param.iteration)
        self.param.iteration += 1
        return -reward

    def clean_lists(self):
        self.lists = Lists()

    def return_lists(self):
        return self.lists

    def shutdown(self):
        input('Press enter to finish ...')
        self.pyrep.shutdown()  # Close the application


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
