from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError

SCENE_FILE = join(dirname(abspath(__file__)), 'petanque.ttt')
pr = PyRep()
pr.launch(SCENE_FILE, headless=False)  # Launch the application with a scene file that contains a robot


class MyRobot(object):  # Definir la estructura del robot
    def __init__(self, my_robot_arm, my_robot_gripper, my_robot_tip):
        self.arm = my_robot_arm
        self.gripper = my_robot_gripper
        self.tip = my_robot_tip
        self.pos = self.arm.get_position()


arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the gripper
tip = Dummy('Panda_tip')
my_panda = MyRobot(arm, gripper, tip)  # Create the robot structure


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self):
        self.sphere = Shape('Sphere')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.target = Shape('target')
        self.success = ProximitySensor('success')


task = InitTask()

# Iniciar la simulacion
pr.start()

# Llevar al robot a la posicion inicial
try:
    path = my_panda.arm.get_path(position=task.wp0.get_position(),
                                 euler=task.wp0.get_orientation())
    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()
except ConfigurationPathError as e:
    print('Could not find path')
    exit()

joint_pos = my_panda.arm.get_joint_positions()
print(joint_pos)
next_joint_pos = joint_pos + np.array([0, 0, 0, 0, 0, -np.pi / 4, 0])
obj_joint_pos = joint_pos + np.array([0, 0, 0, 0, 0, np.pi / 4, 0])
my_panda.arm.set_joint_positions(next_joint_pos)
pr.step()

joint_velocities = np.array([0, 0, 0, 0, 0, 1, 0])
my_panda.arm.set_joint_target_velocities(joint_velocities)
pr.step()

pr.stop()
pr.shutdown()
