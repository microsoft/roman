import numpy as np
import os
import pybullet as pb

################################################################
## Provides access to the sim environment.
################################################################
class SimScene:
    def __init__(self, robot, scene_setup_fn=None, data_dir=None, tex_dir=None, **kwargs):
        if not robot.use_sim:
            raise ValueError("Simulated scenes can only be used with a simulated robot.")
        self._cameras = {}
        self.__light_position = [10, 10, 10]
        self._scene_setup_fn = scene_setup_fn
        self.robot = robot
        self.__tag_map = {}
        self.data_dir = data_dir
        if tex_dir:
            files = os.listdir(tex_dir)
            self.textures = [os.path.join(tex_dir, f) for f in files]

        self.__connected = False

    def connect(self):
        pb.connect(pb.SHARED_MEMORY)
        if self.data_dir:
            pb.setAdditionalSearchPath(self.data_dir)
        self.__connected = True
        return self

    def reset(self):
        if self.__connected:
            self.disconnect()
        self.robot.disconnect()
        self.robot.connect()
        self.connect()
        self.__tag_map = {}
        self._cameras = {}
        self.setup_scene()
        return self.get_world_state()

    def setup_scene(self):
        '''
        Method called after each reset to rebuild the 3D scene.
        Override this method in a derived class to add content to the scene beyond the robot and the table.
        '''
        if self._scene_setup_fn:
            self._scene_setup_fn(self)
        else:
            self.make_table()

    def disconnect(self):
        pb.disconnect()
        self.__connected = False

    def make_table(self, height=0, tex=None, color=(0.2, 0.2, 0.2, 1), **kwargs):
        self.make_box([1, 2, 0.05], [-0.25, 0, height - 0.025], tex=tex, color=color, **kwargs)

    def loadURDF(self,
                 urdf,
                 basePosition=[0, 0, 0],
                 baseOrientation=[0, 0, 0],
                 flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
                 tag=None):
        id = pb.loadURDF(urdf, basePosition=basePosition, baseOrientation=pb.getQuaternionFromEuler(baseOrientation), flags=flags)
        if tag is not None:
            self.__tag_map[id] = tag
        return id

    def make_box(self, size, position=[0, 0, 0], orientation=[0, 0, 0, 1], mass=0, tex=None, color=None, tag=None, **kwargs):
        cid = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=np.array(size) * 0.5)
        vid = pb.createVisualShape(pb.GEOM_BOX, halfExtents=np.array(size) * 0.5)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position, baseOrientation=orientation)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        if tex is not None:
            if type(tex) is str:
                tex = pb.loadTexture(tex)
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if len(kwargs):
            pb.changeDynamics(id, -1, **kwargs)

        if tag is not None:
            self.__tag_map[id] = tag
        return id

    def make_ball(self, radius, position=[0, 0, 0], mass=0, tex=None, color=None, tag=None, **kwargs):
        cid = pb.createCollisionShape(pb.GEOM_SPHERE, radius=radius)
        vid = pb.createVisualShape(pb.GEOM_SPHERE, radius=radius)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        if tex is not None:
            if type(tex) is str:
                tex = pb.loadTexture(tex)
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if len(kwargs):
            pb.changeDynamics(id, -1, **kwargs)
        if tag is not None:
            self.__tag_map[id] = tag
        return id

    def load_obj(self, mesh_file, position, orientation, scale, mass, vhacd_file=None, tex=None, color=None, tag=None,  **kwargs):
        '''Like loadURDF, but simpler and without requiring a urdf file. Supports concave objects (requires a vhacd file).
        Use pb.vhacd(in_mesh_file, out_vhacd_file, log_file, alpha=0.04,resolution=50000 ) to generate a vhacd file. '''

        vhacd_file = vhacd_file if vhacd_file else mesh_file
        cid = pb.createCollisionShape(pb.GEOM_MESH, fileName=vhacd_file, meshScale=scale, collisionFrameOrientation=orientation)
        vid = pb.createVisualShape(pb.GEOM_MESH, fileName=mesh_file, meshScale=scale, visualFrameOrientation=orientation)
        id = pb.createMultiBody(mass, baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, basePosition=position)
        if color is not None:
            pb.changeVisualShape(id, -1, rgbaColor=color)
        if tex is not None:
            if type(tex) is str:
                tex = pb.loadTexture(tex)
            pb.changeVisualShape(id, -1, textureUniqueId=tex)
        if len(kwargs):
            pb.changeDynamics(id, -1, **kwargs)
        if tag is not None:
            self.__tag_map[id] = tag
        return id

    def set_light_position(self, light_position):
        self.__light_position = light_position

    def create_camera(self,
                      cameraEyePosition,
                      img_res=[84, 84],
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

    def get_camera_count(self):
        return len(self._cameras)

    def get_camera_images(self):
        return list(self.get_camera_image(id) for id in self._cameras.keys())

    def get_camera_image(self, id):
        if id not in self._cameras.keys():
            if type(id) is int and id > 0 and id < len(self._cameras):
                id = self._cameras.keys[id]
            else:
                raise ValueError(f"{id} is not a valid camera identifier.")
        camera_cfg = self._cameras[id]
        img_w = camera_cfg["img_w"]
        img_h = camera_cfg["img_h"]
        img_arr = pb.getCameraImage(
            img_w,
            img_h,
            camera_cfg["viewMatrix"],
            camera_cfg["projectionMatrix"],
            lightDirection=self.__light_position,
            flags=pb.ER_NO_SEGMENTATION_MASK,
            renderer=pb.ER_BULLET_HARDWARE_OPENGL)  # renderer=p.ER_TINY_RENDERER
        #rgb
        rgb = np.reshape(img_arr[2], (img_h, img_w, 4))[:, :, :3]

        # depth
        depth_buffer = np.reshape(img_arr[3], (img_h, img_w))
        far = camera_cfg["far"]
        near = camera_cfg["near"]
        depth = far * near / (far - (far - near) * depth_buffer)

        # segmentation mask
        # TBD

        #return (rgb, depth)
        return rgb

    def get_world_state(self):
        '''Enumerates and returns the state of the objects that have a tag.'''
        state = dict()
        for id, tag in self.__tag_map.items():
            state[tag] = self._get_object_state(id)
        return state

    def _get_object_state(self, id):
        pos, orn = pb.getBasePositionAndOrientation(id)
        size = pb.getVisualShapeData(id)[0][3]
        orn = pb.getEulerFromQuaternion(orn)
        lv, av = pb.getBaseVelocity(id)
        return {"position": np.array(pos),
                "size": np.array(size),
                "orientation": np.array(orn),
                "lin_vel": np.array(lv),
                "ang_vel": np.array(av)}
