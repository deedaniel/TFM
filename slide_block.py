import math
from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError

SCENE_FILE = join(dirname(abspath(__file__)), 'slide_block.ttt')
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
        self.block = Shape('block')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.target = Shape('target')
        self.success = ProximitySensor('success')


task = InitTask()

# Declaración y definición de los parametros entrada: distancia y orientación
distance = 0.2
orientation = 0

final_target_rel = np.array([distance*math.sin(orientation), distance*math.cos(orientation), 0])
final_target_abs = final_target_rel + task.wp0.get_position()

final_pos = Dummy.create()
final_pos.set_position(final_target_abs)
final_pos.set_orientation(task.wp0.get_orientation())

tray = [task.wp0, final_pos]

# Ejecución de la trayectoria
pr.start()

time = 0
reward = 0

done = False
# Close the gripper at a velocity of 0.05.
while not done:
    done = gripper.actuate(0, velocity=0.05)
    pr.step()

for pos in tray:
    try:
        path = robot.arm.get_path(position=pos.get_position(),
                                  euler=pos.get_orientation())
        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()
            time += pr.get_simulation_timestep()
            print(robot.tip.get_position())

            distance_objective_3D = np.array(task.wp1.get_position() - robot.tip.get_position())
            distance_objective = np.linalg.norm(distance_objective_3D)

            reward += math.exp(-distance_objective)
    except ConfigurationPathError as e:
        print('Could not find path')
        exit()

pr.stop()
pr.shutdown()
print(time)
print(reward)
