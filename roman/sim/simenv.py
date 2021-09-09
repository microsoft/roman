import pybullet as pb
import os
import math
from . import ur
from . import rq

################################################################
## configures the simulator
################################################################
class SimEnv():
    def __init__(self, config={}):
        self.__urdf = config.get('sim.urdf', 'ur_rq3.urdf')
        self.__base_joint_id = config.get('sim.base_joint_id', 1)
        self.__tcp_id = config.get('sim.tcp_id', 23)
        self.__time_step = config.get('sim.time_step', 1. / 240)
        self._useGUI = config.get('sim.use_gui', True)
        self.__time = 0.0
        self.__cameras = []
        self.__robot_id = None
        if not os.path.exists(self.__urdf):
            self.__urdf = os.path.join(os.path.dirname(__file__), self.__urdf)

    def connect(self):
        if self._useGUI:
            pb.connect(pb.GUI_SERVER)
            pb.resetDebugVisualizerCamera(1.5, -30, -15, cameraTargetPosition=[-0.4, 0, 0.3])
            pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, self._useGUI)
        else:
            pb.connect(pb.SHARED_MEMORY_SERVER)
        self.reset()

    def disconnect(self):
        pb.disconnect()

    def update(self):
        pb.stepSimulation()
        self.__time += self.__time_step

    def time(self):
        return self.__time

    def reset(self):
        pb.resetSimulation()
        pb.setGravity(0, 0, -10)
        self.__robot_id = pb.loadURDF(
            self.__urdf,
            basePosition=[0, 0, 0],
            baseOrientation=pb.getQuaternionFromEuler([0, 0, math.pi]),
            flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
        self.arm = ur.URArm(self.__robot_id, self.__base_joint_id, self.__tcp_id, self.__time_step)
        self.hand = rq.Robotiq3FGripper(self.__robot_id)
        pb.stepSimulation()
        self.arm.reset()
        self.hand.reset()

