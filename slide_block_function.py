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
import matplotlib.pyplot as plt

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


class SlideBlockFunction(object):
    def __init__(self):
        self.pyrep = PyRep()
        self.pyrep.launch(join(DIR_PATH, TTT_FILE), headless=True)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask()

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

        time = 0
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
                    time += self.pyrep.get_simulation_timestep()
                    print(self.robot.tip.get_position())

                    distance_objective_3d = np.array(self.task.wp1.get_position() - self.task.block.get_position())
                    distance_objective = np.linalg.norm(distance_objective_3d)

                    reward += math.exp(-distance_objective)
            except ConfigurationPathError as e:
                print('Could not find path')
                reward = 0
                return -reward

            self.pyrep.stop()  # Stop the simulation
            return -reward
