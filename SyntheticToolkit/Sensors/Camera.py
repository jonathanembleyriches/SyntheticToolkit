import pathlib
from typing import Any, Dict
import numpy as np
import omni.graph.core as og
import omni.replicator.core as rep
from omni.replicator.core.scripts.annotators import Annotator
from PIL import Image


class DepthCamera():
    def __init__(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        image_size=(512, 512),
        attach=True,
        parent="/World/DepthCamera",
        name="DepthCamera",
    ) -> None:
        self._rgb_annot: Annotator
        self._save_path = ""
        self._pos = position
        self._rot = rotation
        self._image_size = image_size
        self._attach = attach
        self._name = name
        self._focal_length = 24.0
        self._focus_distance = 400.0
        self._f_stop = 0.0
        self._horizontal_aperture = 20.955
        self._horizontal_aperture_offset = 0.0
        self._vertical_aperture_offset = 0.0
        self._clipping_range = (1.0, 10000000.0)
        self._resolution = (512, 512)
        self.sample_count = 0
        self.save_path = None
        self._o = "[DepthCamera] "

    def init_output_folder(self, path):
        """
        Initialise the output folder for this sensor relative to the

        :param path: [TODO:description]
        :type path: [TODO:type]
        """
        self.save_path = path
        print(f"{self._o} Initializing output folders")
        pathlib.Path(path + "/camera").mkdir(parents=True, exist_ok=True)

        pathlib.Path(path + "/cameraDepth").mkdir(parents=True, exist_ok=True)
        pathlib.Path(path + "/cameraLabels").mkdir(parents=True, exist_ok=True)
        pathlib.Path(path + "/cameraPC").mkdir(parents=True, exist_ok=True)

    def init_ros(self):
        pass

    def init_sensor(self, parent):
        # print(self.__clipping_range)
        self._cam = rep.create.camera(
            position=self._pos,
            parent=parent,
            name=self._name,
            rotation=self._rot,
            focal_length=self._focal_length,
            focus_distance=self._focus_distance,
            f_stop=self._f_stop,
            horizontal_aperture=self._horizontal_aperture,
            horizontal_aperture_offset=self._horizontal_aperture_offset,
            vertical_aperture_offset=self._vertical_aperture_offset,
            clipping_range=self._clipping_range,
        )
        # print("resolution ", self.__resolution)
        self._rp: og.Node = rep.create.render_product(self._cam, self._resolution)
        # print(f"{self._o} Attaching annotaors to camera.")
        if self._attach:
            self._init_annotators()
            self._attach_annotoators()

    def __del__(self):
        self._detatch_annototators()

    def read_from_json(self, data):
        # We have been given data["LIDAR"]
        # for instance_ids in data:
        camera_settings = data
        self._name = camera_settings["name"]
        self._focal_length = camera_settings["focal_length"]
        self._focus_distance = camera_settings["focus_distance"]
        self._f_stop = camera_settings["f_stop"]
        self._horizontal_aperture = camera_settings["horizontal_aperture"]
        self._horizontal_aperture_offset = camera_settings["horizontal_aperture_offset"]
        self._vertical_aperture_offset = camera_settings["vertical_aperture_offset"]
        self._clipping_range = (
            camera_settings["clipping_range"][0],
            camera_settings["clipping_range"][1],
        )
        self._resolution = camera_settings["resolution"]
        self._pos = camera_settings["position"]
        self._rot = camera_settings["rotation"]

    def construct_pc(self, rgb_image, depth_image):
        pass

    def _init_annotators(self):
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        # self.pc_annot = rep.AnnotatorRegistry.get_annotator("pointcloud")
        self.sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")

    def _attach_annotoators(self):
        self.depth_annot.attach(self._rp)
        self.rgb_annot.attach(self._rp)
        self.sem_annot.attach(self._rp)
        # self.pc_annot.attach(self.__rp)

    def _detatch_annototators(self):
        self.depth_annot.detach(self._rp)
        self.rgb_annot.detach(self._rp)
        self.sem_annot.detach(self._rp)
        # self.pc_annot.dettach(self.__rp)

    def __sample_sensor(self):
        # return
        # await rep.orchestrator.step_async()

        rgb_data = self.rgb_annot.get_data()
        np.save(f"{self.save_path}camera/{self.sample_count}.npy", rgb_data)
        # print(rgb_data)
        im = Image.fromarray(rgb_data, "RGBA")
        path = f"{self.save_path}camera/{self.sample_count}_img.png"
        im.save(path)

        depth_data = self.depth_annot.get_data()

        np.save(f"{self.save_path}cameraDepth/{self.sample_count}.npy", depth_data)
        # np.save('/home/jon/Documents/temp/depth.npy', depth_data)

        sem_data = self.sem_annot.get_data()

        np.save(f"{self.save_path}cameraLabels/{self.sample_count}.npy", sem_data)

        # pc_data = self.pc_annot.get_data()
        # np.save(f"{self.save_path}cameraPC/{self.sample_count}.npy",pc_data)
        self.sample_count += 1
        # np.save('/home/jon/Documents/temp/sem.npy', sem_data)
        return

    def sample_sensor(self) -> Dict[str, Any]:
        """
        Samples the camera sensors and returns all initialised annotated data.

        :return: Dict[str, Any] camera_output_data
        """

        rgb_data = self.rgb_annot.get_data()
        depth = self.depth_annot.get_data()
        camera_output_data = {"rgb": rgb_data, "depth": depth}

        self.sample_count += 1
        return camera_output_data
