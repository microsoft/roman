
import os
import math
from . import ur
from . import rq
import pybullet as pb
import pybullet_data

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
        self._arm_pos = config.get('sim.start_config', [0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0])
        self.__time = 0.0
        self.__cameras = []
        self.__robot_id = None

        if not os.path.exists(self.__urdf):
            self.__urdf = os.path.join(os.path.dirname(__file__), self.__urdf)

    def connect(self):
        if self._useGUI:
            pb.connect(pb.GUI_SERVER)
            pb.resetDebugVisualizerCamera(1.5, -30, -15, cameraTargetPosition=[-0.4, 0, 0.3])
        else:
            pb.connect(pb.SHARED_MEMORY_SERVER)
            pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
        self.__load_robot()

        if self._useGUI:
            pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    def disconnect(self):
        pb.disconnect()

    def update(self):
        pb.stepSimulation()
        self.__time += self.__time_step

    def time(self):
        return self.__time

    def reset(self):
        if self._useGUI:
            pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
        pb.resetSimulation()
        self.__load_robot()
        if self._useGUI:
            pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    def __load_robot(self):
        self.__robot_id = pb.loadURDF(
            self.__urdf,
            basePosition=[0, 0, 0],
            baseOrientation=pb.getQuaternionFromEuler([0, 0, math.pi]),
            flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

        self.arm = ur.URArm(self.__robot_id, self.__base_joint_id, self.__tcp_id, self.__time_step)
        self.hand = rq.Robotiq3FGripper(self.__robot_id)

        # must set gravity before reseting the arm, to allow calibration of the ft sensor
        pb.setGravity(0, 0, -10) # need
        self.arm.reset(self._arm_pos)
        self.hand.reset()

