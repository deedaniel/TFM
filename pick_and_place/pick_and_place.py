from os.path import dirname, join, abspath
import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.errors import ConfigurationPathError

SCENE_FILE = join(dirname(abspath(__file__)), 'pick_and_place.ttt')
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
        self.pick = Shape('large_container')
        self.place = Shape('small_container')
        self.wp0 = Dummy('waypoint0')
        self.wp1 = Dummy('waypoint1')
        self.wp2 = Dummy('waypoint2')
        self.wp3 = Dummy('waypoint3')
        self.success = ProximitySensor('success')


task = InitTask()

# Declaración y definición de los parametros entrada: distancia y orientación
pick_pos_rel = np.array([0, 0, 0])
place_pos_rel = np.array([0, 0, 0.2])

pick_pos = task.pick.get_position() + pick_pos_rel
place_pos = task.place.get_position() + place_pos_rel

task.wp1.set_position(pick_pos)
task.wp3.set_position(place_pos)

tray = [task.wp0, task.wp1, task.wp2, task.wp3]

# Ejecución de la trayectoria
pr.start()
reward = 0

for pos in tray:
    try:
        path = robot.arm.get_path(position=pos.get_position(),
                                  euler=pos.get_orientation())
        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            pr.step()

            distance_objective_3D = np.array(task.place.get_position() - task.block.get_position())
            distance_objective = np.linalg.norm(distance_objective_3D)

            reward += -distance_objective**2

        if pos == task.wp1:
            done = False
            # Open the gripper halfway at a velocity of 0.04.
            while not done:
                done = robot.gripper.actuate(0, velocity=0.04)
                pr.step()
            grasp = robot.gripper.grasp(task.block)
            print(grasp)
        elif pos == task.wp3:
            done = False
            # Open the gripper halfway at a velocity of 0.04.
            while not done:
                done = gripper.actuate(1, velocity=0.04)
                pr.step()
            release = robot.gripper.release()
    except ConfigurationPathError as e:
        reward = -40
        print('Could not find path')
        exit()

pr.stop()
pr.shutdown()
print(reward)
