import math
from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_with_panda_2.ttt')
pr = PyRep()
pr.launch(SCENE_FILE)  # Launch the application with a scene file that contains a robot


class MyRobot(object):  # Define the structure of the robot
    def __init__(self, my_robot_arm, my_robot_gripper):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.pos = self.arm.get_position()


arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the gripper
my_panda = MyRobot(arm, gripper)  # Create the robot structure


class MyObstacle(object):  # Define the structure of the obstacle
    def __init__(self):
        self.obstacle = Shape('obstacle')
        self.pos = self.obstacle.get_position()  # We get the position of the obstacle
        self.radius = 0.3  # We get the radius of the obstacle


obstacle = MyObstacle()

initial_pos = Dummy('target0')
final_pos = Dummy('target1')

pos1_rel = np.array([-0.13855742, -0.27029802,  0.01403834])  # Parametro de entrada: posición relativa al obstaculo
pos1_abs = pos1_rel + obstacle.pos
waypoint1 = Dummy.create()
waypoint1.set_position(pos1_abs)

pos2_rel = np.array([-0.02478715, -0.25553736,  0.16360755])  # Parametro de entrada: posición relativa al obstaculo
pos2_abs = pos2_rel + obstacle.pos
waypoint2 = Dummy.create()
waypoint2.set_position(pos2_abs)

# Calcular distancia de los waypoints al obstaculo

distance_w1_3D = np.array(obstacle.pos - waypoint1.get_position())
distance_w1 = np.linalg.norm(distance_w1_3D)

distance_w2_3D = np.array(obstacle.pos - waypoint2.get_position())
distance_w2 = np.linalg.norm(distance_w2_3D)

if distance_w1 < 0.3 or distance_w2 < 0.3:
    reward = -1000
    print(reward)
    exit()

tray = [initial_pos, waypoint1, waypoint2, final_pos]

# Ejecución de la trayectoria
pr.start()

time = 0
for pos in tray:
    try:
        path = my_panda.arm.get_path(position=pos.get_position(),
                                     euler=[0, math.radians(180), 0])
        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()
            time += pr.get_simulation_timestep()
    except ConfigurationPathError as e:
        print('Could not find path')
        exit()

pr.stop()
pr.shutdown()
print(distance_w1)
print(distance_w2)
print(time)

reward = (-(5 * (1 - obstacle.radius / distance_w1)) ** 2
          - (5 * (1 - obstacle.radius / distance_w2)) ** 2 + 2) + 10 / time
print(reward)
