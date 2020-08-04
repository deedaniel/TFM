from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.objects import ProximitySensor
from pyrep.robots.arms.panda import Panda
from pyrep.objects.dummy import Dummy
from pyrep.errors import ConfigurationPathError
import math
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

SCENE_FILE = join(dirname(abspath(__file__)), 'scene_with_panda_1.ttt')
pr = PyRep()
pr.launch(SCENE_FILE)  # Launch the application with a scene file that contains a robot
pr.start()  # Start the simulation

arm = Panda()  # Get the panda from the scene
gripper = PandaGripper() # Get the gripper
sensor = ProximitySensor('Panda_sensingNose')


class MyRobot(object):  # Define the structure of the robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_panda_sensor):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.sensor = my_panda_sensor


my_panda = MyRobot(arm, gripper, sensor)  # Create the robot structure
robot_pos = my_panda.arm.get_position()


pos = Dummy('target0')  # We get the initial point of the trajectory
dest = Dummy('target1')  # We got the final point of the trajectory

# We are doing a lineal trajectory in small steps
# We define the length of every step
delta = 0.05  # 5 centimeters

# Get a path to the first target (rotate so z points down)
try:
    path = my_panda.arm.get_path(
        position=pos.get_position(), euler=[0, math.radians(180), 0])
except ConfigurationPathError as e:
    print('Could not find path')
    exit()

# Step the simulation and advance the agent along the path
done = False
while not done:
    done = path.step()
    pr.step()
print('Reached target!')

# Define the number of steps in which we'll execute the trajectory
steps = 15
time = 0

for step in range(steps):
    # Calculate the next target
    next_pos = pos.get_position() + np.array([0, delta, 0])
    pos.set_position(next_pos)

    try:
        path = my_panda.arm.get_path(
            position=pos.get_position(), euler=[0, math.radians(180), 0], ignore_collisions=True)
    except ConfigurationPathError as e:
        print('Could not find path')
        continue

    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()
        time += pr.get_simulation_timestep()

    distance_3D = np.array(dest.get_position() - pos.get_position())
    distance = np.linalg.norm(distance_3D)
    print(distance)

print('Done ...')
input('Press enter to finish ...')
pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
