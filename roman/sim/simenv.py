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
    TIME_STEP = 1 / 240.

    '''
    Loads the default pybullet environment and allows further configuration of the cameras, objects present in the scene etc.
    '''
    def __init__(self, useGUI=True):
        self._useGUI = useGUI
        self.__time = 0.0
        self.__cameras = []

    def connect(self):
        if self._useGUI:
            pb.connect(pb.GUI)
            pb.resetDebugVisualizerCamera(1.5, -30, -15, cameraTargetPosition=[-0.4, 0, 0.3])
            pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, self._useGUI)
        else:
            pb.connect(pb.DIRECT)
        self.reset()

    def reset(self):
        pb.resetSimulation()
        pb.setGravity(0, 0, -10)

    def update(self):
        pb.stepSimulation()
        self.__time += SimEnv.TIME_STEP

    def time(self):
        return self.__time

    def disconnect(self):
        pb.disconnect()

    def loadURDF(self, urdf, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0]):
        return pb.loadURDF(urdf, basePosition=basePosition, baseOrientation=pb.getQuaternionFromEuler(baseOrientation), flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS )

    def make_box(self, size, position=[0, 0, 0], orientation=[0, 0, 0, 1], mass=0, tex=None, color=None, restitution=1):
        cid = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=np.array(size) * 0.5)
        vid = pb.createVisualShape(pb.GEOM_BOX, halfExtents=np.array(size) * 0.5)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position, baseOrientation=orientation)
        if tex is not None:
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        pb.changeDynamics(id, -1, restitution=restitution)
        return id

    def make_ball(self, radius, position=[0, 0, 0], mass=0, tex=None, color=None, restitution=1):
        cid = pb.createCollisionShape(pb.GEOM_SPHERE, radius=radius)
        vid = pb.createVisualShape(pb.GEOM_SPHERE, radius=radius)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position)
        if tex is not None:
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        pb.changeDynamics(id, -1, restitution=restitution)
        return id

    def load_obj(self, mesh_file, position, orientation, scale, mass, vhacd_file=None, tex=None, color=None):
        '''Like loadURDF, but simpler and without requiring a urdf file. Supports concave objects (requires a vhacd file).
        Use pb.vhacd(in_mesh_file, out_vhacd_file, log_file, alpha=0.04,resolution=50000 ) to generate a vhacd file. '''

        vhacd_file = vhacd_file if vhacd_file else mesh_file
        cid = pb.createCollisionShape(pb.GEOM_MESH, fileName=vhacd_file, meshScale=scale, collisionFrameOrientation=orientation)
        vid = pb.createVisualShape(pb.GEOM_MESH, fileName=mesh_file, meshScale=scale, visualFrameOrientation=orientation)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position)
        if tex is not None:
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        return id

    def set_light_position(self, light_position):
        self.__light_position = light_position

    def create_camera(self, img_w, img_h, cameraEyePosition, cameraTargetPosition=[0, 0, 0.2], cameraUpVector=[0, 0, 1], fov=50, camera_near=0.01, camera_far=100):
        camera_cfg = {}
        camera_cfg["viewMatrix"] = pb.computeViewMatrix(cameraEyePosition=cameraEyePosition, cameraTargetPosition=cameraTargetPosition, cameraUpVector=cameraUpVector)
        camera_cfg["projectionMatrix"] = pb.computeProjectionMatrixFOV(fov, img_w / img_h, camera_near, camera_far)
        camera_cfg["img_w"] = img_w
        camera_cfg["img_h"] = img_h
        self.__cameras += [camera_cfg]
        return len(self.__cameras) - 1

    def get_camera_image(self, camera_id):
        camera_cfg = self.__cameras[camera_id]
        img_arr = pb.getCameraImage(
            camera_cfg["img_w"],
            camera_cfg["img_h"],
            camera_cfg["viewMatrix"] ,
            camera_cfg["projectionMatrix"],
            lightDirection=self.__light_position,
            flags=pb.ER_NO_SEGMENTATION_MASK,
            renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        #rgb
        rgb = np.reshape(img_arr[2], (self.img_h, self.img_w, 4))[:, :, :3]

        # depth
        depth_buffer = np.reshape(img_arr[3], (self.img_h, self.img_w))
        far = self.camera_far
        near = self.camera_near
        depth = far * near / (far - (far - near) * depth_buffer)

        # segmentation mask
        # TBD

        return (rgb, depth)
