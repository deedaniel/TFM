from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

DIR_PATH = dirname(abspath(__file__))


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


class InitTask(object):  # Declaración y definición de los elementos de la tarea
    def __init__(self, variation):
        self.block = Shape('block0')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.wp2 = Dummy('waypoint2')
        self.slide_target = Dummy('slide_target0')
        self.target_wp0 = Dummy('target_wp0')
        if variation == '2block':
            self.target_wp1 = Dummy('target_wp1')


class Lists(object):  # Declaración de las listas para guardar los parametros y las recompensas de la optimizacion
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []


class SlideBlock(object):  # Clase de la función de la tarea de empujar un objeto
    def __init__(self, headless_mode: bool, variation: str):
        # Al inicializar la clase se carga PyRep, se carga la escena en el simulador, se carga el Robot y los objetos
        # de la escena y se inicializan las listas.
        self.pyrep = PyRep()
        self.variation = variation
        self.ttt_file = 'slide_block_' + self.variation + ".ttt"
        self.pyrep.launch(join(DIR_PATH, self.ttt_file), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask(variation)
        self.lists = Lists()

    def slide_block(self, slide_params: np.array):
        # Definición del punto de empuje
        wp1_pos_rel = np.array([slide_params[0], slide_params[1], slide_params[2]])
        wp1_pos_abs = wp1_pos_rel + self.task.wp0.get_position()
        wp1_or_rel = np.array([0.0, 0.0, slide_params[4]])
        wp1_or_abs = wp1_or_rel + self.task.wp0.get_orientation()
        self.task.wp1.set_position(wp1_pos_abs)
        self.task.wp1.set_orientation(wp1_or_abs)

        # Definición del objetivo final
        distance = slide_params[3]
        orientation = slide_params[4]

        final_pos_rel = np.array([distance * math.sin(orientation), distance * math.cos(orientation), 0])
        final_pos_abs = final_pos_rel + self.task.wp1.get_position()
        final_or_abs = wp1_or_abs
        self.task.wp2.set_position(final_pos_abs)
        self.task.wp2.set_orientation(final_or_abs)

        # Definicion de la trayectoria del robot
        tray = [self.task.wp0, self.task.wp1, self.task.wp2]

        # Iniciar la simulacion
        self.pyrep.start()

        # Se declaran los parametros de la recompensa
        distance_slide = 0.0
        distance_target0 = 0.0
        distance_target1 = 0.0
        or_target0 = 0.0
        or_target1 = 0.0

        # Para la primera variación, se cierra la pinza para poder empujar el objeto.
        if self.variation == '1block':
            done = False
            while not done:
                done = self.robot.gripper.actuate(0, velocity=0.05)
                self.pyrep.step()
        # Ejecución de la trayectoria
        for pos in tray:
            try:
                path = self.robot.arm.get_path(position=pos.get_position(),
                                               euler=pos.get_orientation())
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pyrep.step()

                if pos == self.task.wp1:  # En WP1 se calcula el parametro d_slide
                    distance_slide = self.robot.gripper.check_distance(self.task.block)
                elif pos == self.task.wp2:  # En WP2 se calcula el parametro d_slide y el error de orientación
                    distance_target0 = calc_distance(self.task.block.get_position(),
                                                     self.task.target_wp0.get_position())
                    or_target0 = self.task.block.get_orientation()[2] - self.task.target_wp0.get_orientation()[2]
                    if self.variation == '2block':  # En la 2da variacion se calculan los parametros para 2 soluciones
                        distance_target1 = calc_distance(self.task.block.get_position(),
                                                         self.task.target_wp1.get_position())
                        or_target1 = self.task.block.get_orientation()[2] - self.task.target_wp1.get_orientation()[2]
            except ConfigurationPathError:
                # Si no se encuentra una configuracion para la trayectoria con los waypoints correspondientes
                # se asigna una recompensa de -85
                print('Could not find path')
                reward = -85
                self.pyrep.stop()  # Stop the simulation
                self.lists.list_of_rewards.append(reward)
                self.lists.list_of_parameters.append(list(slide_params))
                return -reward
        # Calculo de la recompensa
        reward = - (10 * distance ** 2 + 200 * distance_slide ** 2 + 200 * distance_target0 ** 2
                    + 400 * distance_target1 ** 2 + 3500 * distance_target0 * distance_target1
                    + 200 * np.abs(or_target0) * distance_target1 + 500 * np.abs(or_target1) * distance_target0)

        self.pyrep.stop()  # Parar la simulación
        self.lists.list_of_rewards.append(reward)  # Se guarda la recompensa del episodio
        self.lists.list_of_parameters.append(list(slide_params)) # Se guardan los parametros del episodio
        return -reward

    def clean_lists(self):
        self.lists = Lists()  # Se limpian las listas

    def return_lists(self):
        return self.lists  # Devolver las listas

    def shutdown(self):
        self.pyrep.shutdown()  # Close the application


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
