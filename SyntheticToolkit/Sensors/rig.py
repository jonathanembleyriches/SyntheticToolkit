from SyntheticToolkit.core.dynamic_objects import DynamicObject


import json

# pxr usd imports used to create cube

from SyntheticToolkit.Sensors.Camera import DepthCamera
from SyntheticToolkit.Sensors.IMU import IMUSensor
from SyntheticToolkit.Sensors.LIDAR import Lidar


class Rig(DynamicObject):
    def __init__(
        self,
        position,
        orientation,
        scale,
        prim_name,
        parent_path,
        stage,
        usd_path=None,
        semantic_class="None",
        instanceable=False,
        visibility="inherited",
        disable_gravity=True,
        scale_delta=0,
    ) -> None:
        super().__init__(
            position,
            orientation,
            scale,
            prim_name,
            parent_path,
            stage,
            usd_path,
            semantic_class,
            instanceable,
            visibility,
            disable_gravity,
            scale_delta,
        )

        self._sensors = []
        self._o = "[SensorRig] "

    
    def compute_observations(self):
        out = []
        for s in self._sensors:
            out.append(s. sample_sensor())
        return out

    def create_rig_from_file(
        self,
        path,
    ):
        pos, ori = self.load_sensors_from_file(path, self._stage)

    def add_sensor_to_rig(self, sensor):
        self._sensors.append(sensor)
        self._sensors[-1].init_sensor(self._full_prim_path)

    def load_sensors_from_file(self, file_path, stage):
        with open(file_path, "r+") as infile:
            print(f"{self._o} Loading sensor rig from file at {file_path}.")
            data = json.load(infile)
            # print(data)
            pos = data["POSITION"]
            ori = data["ORIENTATION"]
            self.velocity = data["VELOCITY"]
            self.sample_rate = data["SAMPLE_RATE"]

            sensors = data["SENSORS"]
            print(sensors)
            for key in sensors:
                if key == "LIDAR":
                    for sensor_id in sensors[key]["instances"]:
                        sensor_settings = sensors[key]["instances"][sensor_id]
                        lidar = Lidar()
                        lidar.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(lidar)
                elif key == "CAMERA":
                    for sensor_id in sensors[key]["instances"]:
                        sensor_settings = sensors[key]["instances"][sensor_id]
                        cam = DepthCamera()
                        cam.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(cam)
                elif key == "IMU":
                    for sensor_id in sensors[key]["instances"]:
                        sensor_settings = sensors[key]["instances"][sensor_id]
                        imu = IMUSensor()
                        imu.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(imu)
                else:
                    print(" ERROR, tried adding sensor with type ", key)
            return pos, ori
