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
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the gripper
tip = Dummy('Panda_tip')
my_panda = MyRobot(arm, gripper, tip)  # Create the robot structure


class MyObstacle(object):  # Define the structure of the obstacle
    def __init__(self):
        self.obstacle = Shape('obstacle')
        self.pos = self.obstacle.get_position()  # We get the position of the obstacle
        self.radius = 0.3  # We get the radius of the obstacle


obstacle = MyObstacle()

initial_pos = Dummy('target0')
final_pos = Dummy('target1')

pos1_rel = np.array([0.3, 0, 0])  # Parametro de entrada: posición relativa al obstaculo
pos1_abs = pos1_rel + obstacle.pos
waypoint1 = Dummy.create()
waypoint1.set_position(pos1_abs)

pos2_rel = np.array([0, 0, 0.3])  # Parametro de entrada: posición relativa al obstaculo
pos2_abs = pos2_rel + obstacle.pos
waypoint2 = Dummy.create()
waypoint2.set_position(pos2_abs)

tray = [initial_pos, waypoint1, waypoint2, final_pos]

# Ejecución de la trayectoria
pr.start()

time = 0
reward = 0
cost = 0
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
            print(my_panda.tip.get_position())

            distance_obstacle_3D = np.array(obstacle.pos - my_panda.tip.get_position())
            distance_obstacle = np.linalg.norm(distance_obstacle_3D)

            distance_objective_3D = np.array(final_pos.get_position() - my_panda.tip.get_position())
            distance_objective = np.linalg.norm(distance_objective_3D)

            reward += -0.1*math.exp(-50*(distance_obstacle-0.3)) + math.exp(-distance_objective)
            cost += 0.1*math.exp(-50*(distance_obstacle-0.3)) + math.exp(distance_objective)
    except ConfigurationPathError as e:
        print('Could not find path')
        exit()

pr.stop()
pr.shutdown()
print(time)
print(reward)
print(cost)
