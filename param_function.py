from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
import matplotlib.pyplot as plt

DIR_PATH = dirname(abspath(__file__))
TTT_FILE = 'scene_with_panda_2.ttt'


class Robot(object):  # Estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


class Target(object):  # Estructura del objetivo
    def __init__(self):
        self.dummy = Dummy('target0')
        self.initial_pos = self.dummy.get_position()
        self.pos = []


class Waypoints(object):
    def __init__(self):
        self.initial_pos = Dummy('target0')
        self.final_pos = Dummy('target1')


class Obstacle(object):  # Estructura del obstaculo
    def __init__(self):
        self.obstacle = Shape('obstacle')
        self.pos = self.obstacle.get_position()  # Posicion del obstaculo
        self.radius = 0.3  # Radio del obstaculo


class Parameters(object):
    def __init__(self):
        self.linear_delta = 0.05  # 5 centimetros
        # Parametros de evitacion: radio y numero de pasos para hacer la trayectoria
        self.radius = 0.3
        self.steps = 10
        self.circular_delta = math.pi / (self.steps - 1)  # 180 / (steps-1) grados para hacer semicircunferencia
        self.time = 0


class ParamFunction(object):
    def __init__(self):
        self.pyrep = PyRep()
        self.pyrep.launch(join(DIR_PATH, TTT_FILE), headless=True)
        self.robot = Robot(Panda(), PandaGripper(), Dummy('Panda_tip'))
        self.obstacle = Obstacle()
        self.target = Target()
        self.param = Parameters()
        self.waypoints = Waypoints()

    def avoidance_tray_circular(self, radius):
        self.pyrep.start()  # We start the simulation

        self.target.pos = self.target.initial_pos  # We set the pos as the initial pos

        # We try to get a path to the initial target of the path
        try:
            path = self.robot.arm.get_path(position=self.target.pos,
                                           euler=[0, math.radians(180), 0])

            # We execute the path
            done = False
            while not done:
                done = path.step()
                self.pyrep.step()
            print('Reached initial target.')
        except ConfigurationPathError:
            print('Could not find path.')
            exit()

        distance_3d = np.array(self.obstacle.pos - self.target.pos)
        distance = np.linalg.norm(distance_3d)

        # Mienstras la distancia al objecto sea menor que el radio del obstaculo, hacemos trayctoria lineal
        while (distance - self.param.linear_delta) > radius:
            # Calcular la siguiente posición
            next_pos = self.target.pos + np.array([0, self.param.linear_delta, 0])
            self.target.pos = next_pos

            try:
                path = self.robot.arm.get_path(position=self.target.pos,
                                               euler=[0, math.radians(180), 0],
                                               ignore_collisions=True)
            except ConfigurationPathError:
                print('Could not find path')
                continue

            # Step the simulation and advance the agent along the path
            done = False
            while not done:
                done = path.step()
                self.pyrep.step()

                distance = calc_distance(self.obstacle.pos, self.target.pos)

        self.param.time = 0  # Inicializamos el tiempo

        # Hacemos trayectoria circular
        for step in range(self.param.steps):
            # Calculate the next target
            next_pos = self.obstacle.pos + np.array([0,
                                                     -radius * math.cos(step * self.param.circular_delta),
                                                     radius * math.sin(step * self.param.circular_delta)])
            self.target.pos = next_pos

            try:
                path = self.robot.arm.get_path(position=self.target.pos,
                                               euler=[0, math.radians(180), 0],
                                               ignore_collisions=True)
            except ConfigurationPathError:
                print('Could not find path')
                continue

            # Step the simulation and advance the agent along the path
            done = False
            while not done:
                done = path.step()
                self.pyrep.step()
                self.param.time += self.pyrep.get_simulation_timestep()

        reward = (-(10 * (1 - self.obstacle.radius / radius)) ** 2 + 2) + 10 / self.param.time
        self.pyrep.stop()  # Stop the simulation
        return -reward

    def avoidance_brute_force(self, radius_interval: list):
        radius_step = 0.005
        list_of_radius = []
        list_of_rewards = []
        self.param.radius = radius_interval[0]

        while self.param.radius <= radius_interval[1]:
            reward = self.avoidance_tray_circular(self.param.radius)
            list_of_radius = np.append(list_of_radius, self.param.radius)
            list_of_rewards = np.append(list_of_rewards, reward)
            self.param.radius += radius_step

        figure, ax = plt.subplots()
        ax.plot(list_of_radius, list_of_rewards)
        ax.set(xlabel='radius (m)', ylabel='reward', title='Reward vs radius')
        ax.grid()
        figure.savefig("plot1.png")
        plt.show()
        print('Done ...')

    def tray_with_waypoints(self, wp_params: np.array):
        # Definicion de los waypoints
        pos1_rel = np.array([wp_params[0], wp_params[1], wp_params[2]])
        pos1_abs = pos1_rel + self.obstacle.pos
        waypoint1 = Dummy.create()
        waypoint1.set_position(pos1_abs)

        pos2_rel = np.array([wp_params[3], wp_params[4], wp_params[5]])
        pos2_abs = pos2_rel + self.obstacle.pos
        waypoint2 = Dummy.create()
        waypoint2.set_position(pos2_abs)

        # Definición de la trayectoria
        tray = [self.waypoints.initial_pos, waypoint1, waypoint2, self.waypoints.final_pos]

        # Ejecución de la trayectoria
        self.pyrep.start()

        self.param.time = 0
        cost = 0
        for pos in tray:
            try:
                path = self.robot.arm.get_path(position=pos.get_position(),
                                               euler=[0, math.radians(180), 0])
                # Step the simulation and advance the agent along the path
                done = False
                while not done:
                    done = path.step()
                    self.pyrep.step()
                    self.param.time += self.pyrep.get_simulation_timestep()

                    distance_obstacle = calc_distance(self.obstacle.pos, self.robot.tip.get_position())
                    distance_objective = calc_distance(self.waypoints.final_pos.get_position(),
                                                       self.robot.tip.get_position())
                    cost += 0.1 * math.exp(-50 * (distance_obstacle - 0.3)) + math.exp(distance_objective)
            except ConfigurationPathError as e:
                cost = 400
                print(cost)
                return cost

        self.pyrep.stop()
        print(cost)
        return cost

    def shutdown(self):
        input('Press enter to finish ...')
        self.pyrep.shutdown()  # Close the application


def calc_distance(vector1: np.array, vector2: np.array):
    distance_3d = np.array(vector1 - vector2)
    distance = np.linalg.norm(distance_3d)
    return distance