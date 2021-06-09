import numpy as np
import pybullet as pb
import os
import math
from . import ur
from . import rq

################################################################
## configures the simulated environment
################################################################
class SimEnv:

    class Camera:
        def __init__(self, camera_fov, light_direction, cameraEyePosition, cameraTargetPosition, cameraUpVector):
            self._videoframe_width = 0
            self._videoframe_height = 0
            self._camera_fov = camera_fov
            self._light = light_direction
            self._cameraEyePosition = cameraEyePosition
            self._cameraTargetPosition = cameraTargetPosition
            self._cameraUpVector = cameraUpVector
            self._viewMatrix = None
            self._projectionMatrix = None
        
        def compute_matrices(self, width, height):
            self._videoframe_width = width
            self._videoframe_height = height
            self._viewMatrix = pb.computeViewMatrix(cameraEyePosition=self._cameraEyePosition, cameraTargetPosition=self._cameraTargetPosition, cameraUpVector=self._cameraUpVector)
            self._projectionMatrix = pb.computeProjectionMatrixFOV(self._camera_fov , self._videoframe_width / self._videoframe_height, nearVal=0.01, farVal=100)
        
        def get_image(self):
            return pb.getCameraImage(self._videoframe_width,
                                     self._videoframe_height,
                                     self._viewMatrix,
                                     self._projectionMatrix,
                                     lightDirection = self._light,
                                     flags = pb.ER_NO_SEGMENTATION_MASK,
                                     renderer=pb.ER_BULLET_HARDWARE_OPENGL)

    TIME_STEP = 1/240.

    '''
    Loads the default pybullet environment and allows further configuration of the cameras, objects present in the scene etc.
    '''
    def __init__(self, useGUI = True):
        self._useGUI = useGUI
        self.__time = 0.0
        self._cameras = []
        # TODO: Measure the settings below, except the projectionMatrix, in the lab.
        light_direction=[10, 10, 10]
        # Overhead camera
        self._cameras.append(SimEnv.Camera(camera_fov=70, light_direction=light_direction, cameraEyePosition=[0,0,2], cameraTargetPosition=[0,0,0], cameraUpVector=[0,1,0]))
        # Side camera
        self._cameras.append(SimEnv.Camera(camera_fov=70, light_direction=light_direction, cameraEyePosition=[-1.0,-0.6, 0.2], cameraTargetPosition=[0,0.2,0], cameraUpVector=[0,0,1]))

    @property
    def num_cams(self):
        print("Getting value...")
        return len(self._cameras)

    def connect(self):
        if self._useGUI:
            pb.connect(pb.GUI)
            pb.resetDebugVisualizerCamera(1.5, -30, -15, cameraTargetPosition=[-0.4, 0, 0.3])
        else:
            pb.connect(pb.DIRECT)
        self.reset()

    def reset(self):
        pb.resetSimulation()
        pb.setGravity(0,0,-10)

    def update(self):
        pb.stepSimulation()
        self.__time += SimEnv.TIME_STEP
        
    def time(self):
        return self.__time
   
    def disconnect(self):
        pb.disconnect()

    def loadURDF(self, urdf, basePosition=[0,0,0], baseOrientation = [0,0,0]):
        return pb.loadURDF(urdf, basePosition = basePosition, baseOrientation=pb.getQuaternionFromEuler(baseOrientation), flags = pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS )

    def make_box(self, size, position=[0,0,0], orientation=[0,0,0,1], mass=0, tex=None, color=None):
        cid = pb.createCollisionShape(pb.GEOM_BOX, halfExtents = np.array(size)*0.5)
        vid = pb.createVisualShape(pb.GEOM_BOX, halfExtents = np.array(size)*0.5)
        id = pb.createMultiBody(mass,baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position, baseOrientation=orientation)
        if tex is not None:
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        #pb.changeDynamics(id, -1, restitution = 0.5)
        return id

    def init_cameras(self, width, height):
        for cam in self._cameras:
            cam.compute_matrices(width, height)

    def get_object_position_and_orientation(self, object_id):
        return pb.getBasePositionAndOrientation(object_id)

    def reset_object_position_and_orientation(self, object_id, position=[0,0,0], orientation=[0,0,0,1]):
        pb.resetBasePositionAndOrientation(object_id, position, orientation)

    def remove_object(self, object_id):
        pb.removeBody(object_id)

    def get_camera_images(self):
        assert pb.isNumpyEnabled(), "Numpy is not enabled: getting images from pybullet will be too slow."
        images = []

        for cam in self._cameras:
            images.append(cam.get_image())

        return images

 