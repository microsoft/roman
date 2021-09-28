from multiprocessing import Value
from typing import KeysView
import numpy as np
import pybullet as pb
import os
import math
from . import ur
from . import rq


################################################################
## Provides access to the sim environment.
################################################################
class SimScene:
    def __init__(self, robot, scene_setup_fn):
        if not robot._use_sim:
            raise ValueError("Simulated scenes can only be used with a simulated robot.")
        self._cameras = {}
        self._scene_setup_fn = scene_setup_fn or SimScene.make_table
        self._server = robot._server
        self.__tag_map = {}

    def connect(self):
        pb.connect(pb.SHARED_MEMORY)
        if self._scene_setup_fn:
            self._scene_setup_fn(self)
        return self

    def reset(self):
        self._server.reset()
        if self._scene_setup_fn:
            self._scene_setup_fn(self)

    def disconnect(self):
        pb.disconnect()

    def make_table(self):
        self.make_box([1, 2, 0.05], [-0.25, 0, -0.025], color=(0.2, 0.2, 0.2, 1))

    def loadURDF(self,
                 urdf,
                 basePosition=[0, 0, 0],
                 baseOrientation=[0, 0, 0],
                 flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
                 tag=None):
        id = pb.loadURDF(urdf, basePosition=basePosition, baseOrientation=pb.getQuaternionFromEuler(baseOrientation), flags=flags)
        if tag:
            self.__tag_map[id] = tag
        return id

    def make_box(self, size, position=[0, 0, 0], orientation=[0, 0, 0, 1], mass=0, tex=None, color=None, restitution=0, tag=None):
        cid = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=np.array(size) * 0.5)
        vid = pb.createVisualShape(pb.GEOM_BOX, halfExtents=np.array(size) * 0.5)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position, baseOrientation=orientation)
        if tex is not None:
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        pb.changeDynamics(id, -1, restitution=restitution)

        if tag:
            self.__tag_map[id] = tag
        return id

    def make_ball(self, radius, position=[0, 0, 0], mass=0, tex=None, color=None, restitution=1, tag=None):
        cid = pb.createCollisionShape(pb.GEOM_SPHERE, radius=radius)
        vid = pb.createVisualShape(pb.GEOM_SPHERE, radius=radius)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position)
        if tex is not None:
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        pb.changeDynamics(id, -1, restitution=restitution)
        if tag:
            self.__tag_map[id] = tag
        return id

    def load_obj(self, mesh_file, position, orientation, scale, mass, vhacd_file=None, tex=None, color=None, restitution=1, tag=None):
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
        pb.changeDynamics(id, -1, restitution=restitution)
        if tag:
            self.__tag_map[id] = tag
        return id

    def set_light_position(self, light_position):
        self.__light_position = light_position

    def create_camera(self,
                      cameraEyePosition,
                      img_res=(84, 84),
                      cameraTargetPosition=[0, 0, 0.2],
                      cameraUpVector=[0, 0, 1],
                      fov=90,
                      camera_near=0.01,
                      camera_far=100,
                      tag=None):
        img_w, img_h = img_res
        camera_cfg = {}
        camera_cfg["viewMatrix"] = pb.computeViewMatrix(cameraEyePosition=cameraEyePosition, cameraTargetPosition=cameraTargetPosition, cameraUpVector=cameraUpVector)
        camera_cfg["projectionMatrix"] = pb.computeProjectionMatrixFOV(fov, img_w / img_h, camera_near, camera_far)
        camera_cfg["img_w"] = img_w
        camera_cfg["img_h"] = img_h
        camera_cfg["near"] = camera_near
        camera_cfg["far"] = camera_far
        tag = tag or len(self.__cameras)
        self._cameras[tag] = camera_cfg
        return tag

    def get_camera_image(self, id):
        if id not in self._cameras.keys and type(id) is int and id > 0 and id < len(self.cameras):
            id = self._cameras.keys[id]
        else:
            raise ValueError(f"{id} is not a valid camera identifier.")
        camera_cfg = self._cameras[id]
        img_arr = pb.getCameraImage(
            camera_cfg["img_w"],
            camera_cfg["img_h"],
            camera_cfg["viewMatrix"],
            camera_cfg["projectionMatrix"],
            lightDirection=self.__light_position,
            flags=pb.ER_NO_SEGMENTATION_MASK,
            renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        #rgb
        rgb = np.reshape(img_arr[2], (self.img_h, self.img_w, 4))[:, :, :3]

        # depth
        depth_buffer = np.reshape(img_arr[3], (self.img_h, self.img_w))
        far = camera_cfg["far"]
        near = camera_cfg["near"]
        depth = far * near / (far - (far - near) * depth_buffer)

        # segmentation mask
        # TBD

        return (rgb, depth)

    def get_state(self):
        '''Enumerates and returns the state of the objects that have a tag.'''
        state = list((tag, self._get_object_state(id)) for id, tag in self.__tag_map.items())
        return state

    def _get_object_state(self, id):
        pos, orn = pb.getBasePositionAndOrientation(id)
        orn = pb.getEulerFromQuaternion(orn)
        lv, av = pb.getBaseVelocity(id)
        return np.array(pos + orn + lv + av)
