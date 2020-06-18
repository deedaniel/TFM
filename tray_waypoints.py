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
pr.launch(SCENE_FILE, headless=True)  # Launch the application with a scene file that contains a robot


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
        self.radius = 0.3/2  # We get the radius of the obstacle


obstacle = MyObstacle()
