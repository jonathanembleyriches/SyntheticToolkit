import numpy as np
import omni
import utils.omni_utils as omni_utils
from SyntheticToolkit.core.objects import BaseObject
from omni.isaac.dynamic_control import _dynamic_control
from pxr import (
    Gf,
)
from scipy.spatial.transform import Rotation as R
from .physxutils import setRigidBody


class DynamicObject(BaseObject):
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

        self._curr_waypoint_id = 0
        self._velocity = 0.25

        self._waypoints = []
        self._full_prim_path = f"{self._prim_path}/{self._prim_name}"
        self._full_prim_path = self._prim_path

        self._stage = omni.usd.get_context().get_stage()

        self.prim = self._stage.GetPrimAtPath(self._full_prim_path)

        self.orient_val = self._prim.GetAttribute("xformOp:orient")
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._rb = self._dc.get_rigid_body(self._full_prim_path)

        self._waypoint_follow_mode = 1  # 0 is free, 1 is only x y

        setRigidBody(self._prim, "convexHull", False)
        self._prim.GetAttribute("physxRigidBody:disableGravity").Set(False)

    def apply_veloc(self, veloc, ang_veloc):
        # print('applying ', veloc)
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        self._dc.set_rigid_body_linear_velocity(self._rb, veloc)

        self._dc.set_rigid_body_angular_velocity(self._rb, ang_veloc)

    def get_pos_rot(self):
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        object_pose = self._dc.get_rigid_body_pose(self._rb)
        return object_pose.p, object_pose.r

    def init_waypoints(self, waypoints):
        if len(waypoints) == 0:
            print("ERROR, no updating as waypoints provided are empty")
            return

        self.reset_waypoints(waypoints)

    def reset_waypoints(self, new_waypoints):
        # Quick fix for just x,y provided
        if len(new_waypoints[0]) == 2:
            for wp in new_waypoints:
                wp.append[0]

        self._curr_waypoint_id = 0
        self._waypoints = new_waypoints

        # self.reset_orient()
        self.apply_veloc([0, 0, 0], [0, 0, 0])

    def move(self):
        if len(self._waypoints) == 0:
            return
        if self._curr_waypoint_id >= len(self._waypoints):
            self._curr_waypoint_id = 0

        # Retrieve the current position and orientation of the sensor rig
        current_pos, current_rot = self.get_pos_rot()
        current_pos = Gf.Vec3d(current_pos[0], current_pos[1], current_pos[2])

        move_vec, rot_float = self._waypoint_update(current_pos)
        # FIX: prevent any other rotations aside from around z
        # temp fix to stop any weird rotations
        if self._waypoint_follow_mode == 1:
            rot_float[0] = 0
            rot_float[1] = 0

        self.apply_veloc(move_vec, rot_float)

    def compute_angular_velocity(self, current_pos, target_pos, current_orient):
        # Vector from current position to target position
        direction_vector = np.array(target_pos) - np.array(current_pos)
        direction_vector = direction_vector / np.linalg.norm(
            direction_vector
        )  # Normalize

        # Convert quaternion to rotation matrix
        rotation_matrix = R.from_quat(current_orient).as_matrix()

        # Assuming the forward vector is along the z-axis
        forward_vector = np.dot(rotation_matrix, np.array([1, 0, 0]))

        # Compute rotation axis and angle via cross product and dot product
        cross_prod = np.cross(forward_vector, direction_vector)
        dot_prod = np.dot(forward_vector, direction_vector)

        # Compute the angle using the dot product (safe for acos)
        angle = np.arccos(np.clip(dot_prod, -1.0, 1.0))

        # Normalize the rotation axis
        norm_cross_prod = np.linalg.norm(cross_prod)
        if norm_cross_prod == 0:
            return np.array([0, 0, 0])  # Already aligned, no rotation needed
        rotation_axis = cross_prod / norm_cross_prod

        # Angular velocity (angle per unit time, around the rotation axis)
        angular_velocity = (
            rotation_axis * angle
        )  # Assuming unit time (e.g., one second)
        # angular_velocity[2] = min(angular_velocity[2], 0.1)
        angular_velocity[2] = max(-0.5, min(angular_velocity[2], 0.5))
        return angular_velocity

    def _waypoint_update(self, pos):
        goal_pos_raw = self._waypoints[self._curr_waypoint_id]
        if self._waypoint_follow_mode:
            pos[2] = 0
            goal_pos_raw[2] = 0

        # goal_pos[2] = z_val
        goal_pos = Gf.Vec3d(goal_pos_raw)
        move_vec = goal_pos - pos
        distance = np.linalg.norm(goal_pos - pos)
        move_vec = (move_vec / distance) * self._velocity

        ori_now = self.orient_val.Get()
        ori_np = list(ori_now.GetImaginary()) + [ori_now.GetReal()]
        orient = ori_now

        orient = self._prim.GetAttribute("xformOp:orient").Get()
        orient = [orient.GetReal()] + list(orient.GetImaginary())
        move_vec = omni_utils.rotate_vector_by_quaternion([3, 0, 0], orient)
        rot_float = self.compute_angular_velocity(pos, goal_pos_raw, ori_np)
        if (
            abs(rot_float[2]) > 0.30
            or abs(rot_float[1]) > 0.30
            or abs(rot_float[0]) > 0.30
        ):
            move_vec = [0, 0, 0]

        if (
            abs(move_vec[2]) > 0.30
            or abs(move_vec[1]) > 0.30
            or abs(move_vec[0]) > 0.30
        ):
            rot_float = [0, 0, 0]

        if distance < 0.5:
            self._curr_waypoint_id += 1

            if self._curr_waypoint_id >= len(self._waypoints):
                self._curr_waypoint_id = 0

            return self._waypoint_update(pos)

        return move_vec, rot_float
