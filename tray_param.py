from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
import matplotlib
import matplotlib.pyplot as plt

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_with_panda_2.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=True)  # Launch the application with a scene file that contains a robot


class MyRobot(object):  # Define the structure of the robot
    def __init__(self, my_robot_arm, my_robot_gripper):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.pos = self.arm.get_position()


arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the gripper
my_panda = MyRobot(arm, gripper)  # Create the robot structure


class MyTarget(object):  # Define the structure of the target
    def __init__(self):
        self.dummy = Dummy('target0')
        self.initial_pos = self.dummy.get_position()
        self.pos = []


target = MyTarget()  # We get the target


class MyObstacle(object):  # Define the structure of the obstacle
    def __init__(self):
        self.obstacle = Shape('obstacle')
        self.pos = self.obstacle.get_position()  # We get the position of the obstacle
        self.radius = 0.3/2  # We get the radius of the obstacle


obstacle = MyObstacle()


class Parameters(object):
    def __init__(self):
        self.linear_delta = 0.05  # 5 centimeters
        # Avoidance parameters: radius and number of steps. Number of steps is fixed, radius changes.
        self.radius = 0.3
        self.steps = 10
        self.circular_delta = math.pi / (self.steps - 1)  # 180 / (steps-1) degrees to make a semi circumference
        self.time = 0
        self.number_of_radius = 5
        self.radius_step = 0.05


param = Parameters()

list_of_radius = []
list_of_rewards = []

for number in range(param.number_of_radius):
    pr.start()  # Start the simulation

    target.pos = target.initial_pos  # We update the pos with the initial position
    # Get a path to the first target (rotate so z points down)
    try:
        path = my_panda.arm.get_path(
            position=target.pos, euler=[0, math.radians(180), 0])

        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()
        print('Reached initial target!')
    except ConfigurationPathError as e:
        print('Could not find path')
        exit()

    distance_3D = np.array(obstacle.pos - target.pos)
    distance = np.linalg.norm(distance_3D)

    while (distance - param.linear_delta) > param.radius:
        # Calculate the next target
        next_pos = target.pos + np.array([0, param.linear_delta, 0])
        target.pos = next_pos

        try:
            path = my_panda.arm.get_path(
                position=target.pos, euler=[0, math.radians(180), 0], ignore_collisions=True)
        except ConfigurationPathError as e:
            print('Could not find path')
            continue

        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()

        distance_3D = np.array(obstacle.pos - target.pos)
        distance = np.linalg.norm(distance_3D)

    for step in range(param.steps):
        # Calculate the next target
        next_pos = obstacle.pos + np.array([0,
                                            -param.radius * math.cos(step * param.circular_delta),
                                            param.radius * math.sin(step * param.circular_delta)])
        target.pos = next_pos

        try:
            path = my_panda.arm.get_path(
                position=target.pos, euler=[0, math.radians(180), 0], ignore_collisions=True)
        except ConfigurationPathError as e:
            print('Could not find path')
            continue

        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()
            param.time += pr.get_simulation_timestep()

    reward = obstacle.radius / param.radius + 10 / param.time

    list_of_radius = np.append(list_of_radius, param.radius)
    list_of_rewards = np.append(list_of_rewards, reward)
    print(list_of_radius)

    param.radius += param.radius_step
    pr.stop()  # Stop the simulation


fig, ax = plt.subplots()
ax.plot(list_of_radius, list_of_rewards)

ax.set(xlabel='radius (m)', ylabel='reward', title='Reward vs radius')
ax.grid()

fig.savefig("plot1.png")
plt.show()

print('Done ...')
input('Press enter to finish ...')

pr.shutdown()  # Close the application
