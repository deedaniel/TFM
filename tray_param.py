from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_with_panda_2.ttt')
pr = PyRep()
pr.launch(SCENE_FILE)  # Launch the application with a scene file that contains a robot
pr.start()  # Start the simulation

arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the gripper


class MyRobot(object):  # Define the structure of the robot
    def __init__(self, my_robot_arm, my_robot_gripper):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper


my_panda = MyRobot(arm, gripper)  # Create the robot structure
robot_pos = my_panda.arm.get_position()

dummy = Dummy('target0')  # We get the initial point of the trajectory
pos = dummy.get_position()

obstacle = Shape('obstacle')  # We get the position of the obstacle
obstacle_pos = obstacle.get_position()
obstacle_radius = 0.3  # 0.3 meters

linear_delta = 0.05  # 5 centimeters

# Avoidance parameters, radius and number of steps
radius = 0.4  # 0.4 meters
steps = 10
circular_delta = math.pi/(steps-1)  # 180 / steps degrees
time = 0

# Get a path to the first target (rotate so z points down)
try:
    path = my_panda.arm.get_path(
        position=pos, euler=[0, math.radians(180), 0])

    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()
    print('Reached initial target!')
except ConfigurationPathError as e:
    print('Could not find path')
    exit()

distance_3D = np.array(obstacle_pos - pos)
distance = np.linalg.norm(distance_3D)
print(distance)

while (distance - linear_delta) > radius:
    # Calculate the next target
    next_pos = pos + np.array([0, linear_delta, 0])
    pos = next_pos

    try:
        path = my_panda.arm.get_path(
            position=pos, euler=[0, math.radians(180), 0], ignore_collisions=True)
    except ConfigurationPathError as e:
        print('Could not find path')
        continue

    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()

    distance_3D = np.array(obstacle_pos - pos)
    distance = np.linalg.norm(distance_3D)

for step in range(steps):
    # Calculate the next target
    next_pos = obstacle_pos + np.array([0,
                                        -radius * math.cos(step * circular_delta),
                                        radius * math.sin(step * circular_delta)])
    pos = next_pos

    try:
        path = my_panda.arm.get_path(
            position=pos, euler=[0, math.radians(180), 0], ignore_collisions=True)
    except ConfigurationPathError as e:
        print('Could not find path')
        continue

    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()
        time += pr.get_simulation_timestep()

reward = obstacle_radius/radius + 10/time
print(reward)

print('Done ...')
input('Press enter to finish ...')
pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
