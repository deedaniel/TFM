from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError

DIR_PATH = dirname(abspath(__file__))


class Robot(object):  # Definicion de la estructura del robot
    def __init__(self, robot_arm, robot_gripper, robot_tip):
        self.arm = robot_arm
        self.gripper = robot_gripper
        self.tip = robot_tip
        self.pos = robot_arm.get_position()


class InitTask(object):  # Declaración y definición de los elementos de la tarea
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


class Lists(object):  # Definición de las listas para guardar datos de las optimizaciones
    def __init__(self):
        self.list_of_parameters = []
        self.list_of_rewards = []


class PickAndPlace(object):  # Declaración de la clase de la tarea de pick and place
    def __init__(self, headless_mode: bool, variation="1container"):
        # Inicialización de la tarea, lanzando PyRep, cargando la escena en el simulador, cargando el robot y los
        # elementos de la escena y inicializando las listas.
        self.pr = PyRep()
        self.variation = variation
        self.ttt_file = 'pick_and_place_' + self.variation + '.ttt'
        self.pr.launch(join(DIR_PATH, self.ttt_file), headless=headless_mode)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.task = InitTask(self.variation)
        self.lists = Lists()

    def pick_and_place(self, wp_params: np.array):
        # Definición de los waypoint de grasp y dejada
        pick_pos_rel = np.array([wp_params[0], wp_params[1], wp_params[2]])
        place_pos_rel = np.array([wp_params[3], wp_params[4], wp_params[5]])

        pick_pos = self.task.wp0.get_position() + pick_pos_rel
        place_pos = self.task.wp2.get_position() + place_pos_rel

        self.task.wp1.set_position(pick_pos)
        self.task.wp3.set_position(place_pos)

        # Definición de la trayectoria
        tray = [self.task.wp0, self.task.wp1, self.task.wp0, self.task.wp2, self.task.wp3, self.task.wp2]

        # Inicio de la simulacion
        self.pr.start()
        self.task.block.set_position(self.task.init_pos)

        # Declaración de los parametros de la recompensa
        distance_pick = 0.0
        distance_tip0 = 0.0
        distance_tip1 = 0.0
        distance_place1 = 0.0

        # Ejecución de la trayectoria.
        for pos in tray:
            try:
                path = self.robot.arm.get_path(position=pos.get_position(),
                                               euler=pos.get_orientation())
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pr.step()

                if pos == self.task.wp1:  # Si estamos en WP1 cerramos la pinza para coger el objeto.
                    done = False
                    while not done:
                        done = self.robot.gripper.actuate(0, velocity=0.04)
                        self.pr.step()
                    self.robot.gripper.grasp(self.task.block)
                    distance_pick = self.robot.tip.check_distance(self.task.pick_wp)
                elif pos == self.task.wp3:  # Si estamos en WP3 abrimos la pinza para dejar el objeto
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
                # Si no se encuentra una configuracion para los waypoints de la trayectoria se asigna una recompensa
                print('Could not find path')
                reward = -50
                if self.variation == "2container":
                    reward = -750
                self.pr.stop()  # Stop the simulation
                self.lists.list_of_parameters.append(wp_params)
                self.lists.list_of_rewards.append(reward)
                return -reward

        distance_place0 = calc_distance(self.task.block.get_position(), self.task.place_wp0.get_position())
        if self.variation == '2container':
            distance_place1 = calc_distance(self.task.block.get_position(), self.task.place_wp1.get_position())
        # Calculo de la recompensa
        reward = - (200 * distance_pick ** 2 + 200 * distance_place0 ** 2 + 400 * distance_place1 ** 2
                    + 3500 * distance_place0 * distance_place1 + 100 * distance_tip0 ** 2 + 200 * distance_tip1 ** 2 +
                    + 1750 * distance_tip0 * distance_tip1)

        self.pr.stop()  # Stop the simulation
        self.lists.list_of_parameters.append(wp_params) # Se guardan los parametros del episodio
        self.lists.list_of_rewards.append(reward)  # Se guarda la recompensa del episodio
        return -reward

    def shutdown(self):
        self.pr.shutdown()  # Close the application

    def clean_lists(self):
        self.lists = Lists()  # Se limpian las listas

    def return_lists(self):
        return self.lists  # Devolver las listas


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance
