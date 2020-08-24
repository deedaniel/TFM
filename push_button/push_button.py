from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.objects.joint import Joint
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError

SCENE_FILE = join(dirname(abspath(__file__)), 'push_button.ttt')
pr = PyRep()
pr.launch(SCENE_FILE)


# Definicion de la estructura del robot
class Robot(object):
    def __init__(self, robot_arm, robot_gripper, robot_tip):
        self.arm = robot_arm
        self.gripper = robot_gripper
        self.tip = robot_tip
        self.pos = robot_arm.get_position()


arm = Panda()
gripper = PandaGripper()
tip = Dummy('Panda_tip')
robot = Robot(arm, gripper, tip)


# Declaración y definición de los elementos de la tarea
class InitTask(object):
    def __init__(self):
        self.target_button = Shape('push_button_target')
        self.target_topPlate = Shape('target_button_topPlate')
        self.target_wrap = Shape('target_button_wrap')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.joint = Joint('target_button_joint')


task = InitTask()


class Parameters(object):
    def __init__(self):
        self.original_pos = task.joint.get_joint_position()


param = Parameters()
print(param.original_pos)

# Declaración y definición de los parametros entrada: distancia y orientación
push_pos_rel = np.array([0, 0, 0.01])

push_pos = task.target_button.get_position() + push_pos_rel

task.wp1.set_position(push_pos)

tray = [task.wp0, task.wp1]

# Ejecución de la trayectoria
pr.start()
reward = 0

done = False
# Close the gripper at a velocity of 0.04.
while not done:
    done = gripper.actuate(0, velocity=0.04)
    pr.step()

for pos in tray:
    try:
        path = robot.arm.get_path(position=pos.get_position(),
                                  euler=pos.get_orientation(),
                                  ignore_collisions=True)
        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()
        reward = 2000 * np.abs(np.linalg.norm(task.joint.get_joint_position() - param.original_pos) - 0.003)
    except ConfigurationPathError as e:
        reward = -40
        print('Could not find path')
        exit()

pr.stop()
pr.shutdown()
print(reward)
