import numpy as np

from omni.physx import get_physx_scene_query_interface

from pxr import Gf, Sdf, Semantics, UsdGeom


def convert_vector_types(in_data, out_type):
    return out_type(in_data)

def convert_quaternion_types(in_data, out_type):
    if out_type == Gf.Quatf or out_type == Gf.Quatd:
        return out_type(in_data[0],in_data[1], in_data[2], in_data[3])

    in_type = type(in_data)
    if in_type == Gf.Quatf or in_type == Gf.Quatd:
        real = in_data.GetReal()
        imaginary = in_data.GetImaginary()
        imaginary = convert_vector_types(imaginary, list)
        full = [real, imaginary]
        return out_type(full)



def ray_cast(origin, direction, distance, query_interface=None, full_details=True):
    if not query_interface:
        query_interface = get_physx_scene_query_interface()

    hit = query_interface.raycast_closest(origin, direction, distance)

    if hit["hit"]:
        distance = hit["distance"]


        return hit

    return None


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qw, qx, qy, qz]

def quaternion_multiply(q1, q2):
    """Multiplies two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )

def quaternion_conjugate(q):
    """Returns the conjugate of a quaternion."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def rotate_vector_by_quaternion(v, q):
    """Rotates a vector v by a quaternion q."""
    # Create a quaternion with zero scalar part from the vector
    vec_quat = np.array([0] + list(v))
    # Quaternion for the vector rotated
    q_vec_rotated = quaternion_multiply(
        quaternion_multiply(q, vec_quat), quaternion_conjugate(q)
    )
    # Return only the vector part
    return q_vec_rotated[1:]

# def euler_from_quaternion(x, y, z, w):
#     """
#     Convert a quaternion into euler angles (roll, pitch, yaw)
#     roll is rotation around x in radians (counterclockwise)
#     pitch is rotation around y in radians (counterclockwise)
#     yaw is rotation around z in radians (counterclockwise)
#     """
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll_x = math.atan2(t0, t1)
#
#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch_y = math.asin(t2)
#
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw_z = math.atan2(t3, t4)
#
#     return roll_x, pitch_y, yaw_z
# def point_in_triangle(self, pt, v1, v2, v3):
#     def sign(p1, p2, p3):
#         return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
#
#     d1 = sign(pt, v1, v2)
#     d2 = sign(pt, v2, v3)
#     d3 = sign(pt, v3, v1)
#
#     has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
#     has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
#
#     return not (has_neg and has_pos)

# def point_in_triangle3(self, pt, A, B, C):
#     xp = pt[0]
#     yp = pt[1]
#
#     (
#         x1,
#         y1,
#         z1,
#     ) = (
#         A[0],
#         A[1],
#         A[2],
#     )
#     (
#         x2,
#         y2,
#         z2,
#     ) = (
#         B[0],
#         B[1],
#         B[2],
#     )
#     (
#         x3,
#         y3,
#         z3,
#     ) = (
#         C[0],
#         C[1],
#         C[2],
#     )
#     c1 = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
#     c2 = (x3 - x2) * (yp - y2) - (y3 - y2) * (xp - x2)
#     c3 = (x1 - x3) * (yp - y3) - (y1 - y3) * (xp - x3)
#     if (c1 < 0 and c2 < 0 and c3 < 0) or (c1 > 0 and c2 > 0 and c3 > 0):
#         return True
#     else:
#         return False

# def point_in_triangle2(self, pt, A, B, C):
#     """
#     Check if point pt lies inside the triangle defined by vertices A, B, C
#     Each vertex is a tuple (x, y, z)
#     """
#     # Convert vertices and point to numpy arrays
#     pt = np.array([pt[0], pt[1], 0])
#     A = np.array([A[0], A[1], 0])
#     B = np.array([B[0], B[1], 0])
#     C = np.array([C[0], C[1], 0])
#
#     # Vectors
#     v0 = C - A
#     v1 = B - A
#     v2 = pt - A
#
#     # Compute dot products
#     dot00 = np.dot(v0, v0)
#     dot01 = np.dot(v0, v1)
#     dot02 = np.dot(v0, v2)
#     dot11 = np.dot(v1, v1)
#     dot12 = np.dot(v1, v2)
#
#     # Compute barycentric coordinates
#     invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
#     u = (dot11 * dot02 - dot01 * dot12) * invDenom
#     v = (dot00 * dot12 - dot01 * dot02) * invDenom
#
#     # Check if point is in triangle
#     return (u >= 0) and (v >= 0) and (u + v < 1)

# def calculate_height(self, x, y, A, B, C):
#     """
#     Calculate the height at point (x, y) within a triangle with vertices A, B, C
#     Each vertex is a tuple (x, y, z)
#     """
#
#     # Convert vertices to numpy arrays for vector operations
#     A = np.array(A)
#     B = np.array(B)
#     C = np.array(C)
#     P = np.array([x, y, 0])  # We don't know the z-coordinate yet
#
#     # Vectors for the edges of the triangle
#     v0 = B - A
#     v1 = C - A
#     v2 = P - A
#
#     # Compute dot products
#     dot00 = np.dot(v0, v0)
#     dot01 = np.dot(v0, v1)
#     dot02 = np.dot(v0, v2)
#     dot11 = np.dot(v1, v1)
#     dot12 = np.dot(v1, v2)
#
#     # Compute barycentric coordinates
#     invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
#     alpha = (dot11 * dot02 - dot01 * dot12) * invDenom
#     beta = (dot00 * dot12 - dot01 * dot02) * invDenom
#     gamma = 1 - alpha - beta
#
#     # Interpolate the height using the barycentric coordinates
#     z = alpha * A[2] + beta * B[2] + gamma * C[2]
#
#     return z

# def barycentric_weights(self, x, y, A, B, C):
#     """
#     Calculate the barycentric weights for a point (x, y) within a triangle defined by vertices A, B, and C.
#     Each vertex is a tuple (x, y, z).
#     """
#
#     def tri_area(P1, P2, P3):
#         return 0.5 * np.linalg.det(
#             [[P1[0], P1[1], 1], [P2[0], P2[1], 1], [P3[0], P3[1], 1]]
#         )
#
#     area_ABC = tri_area(A, B, C)
#     weight_A = tri_area((x, y), B, C) / area_ABC
#     weight_B = tri_area(A, (x, y), C) / area_ABC
#     weight_C = tri_area(A, B, (x, y)) / area_ABC
#
#     return weight_A, weight_B, weight_C
#
# def interpolate_z(self, x, y, A, B, C):
#     """
#     Interpolate the z-value at a point (x, y) inside a triangle defined by vertices A, B, and C.
#     """
#     weight_A, weight_B, weight_C = self.barycentric_weights(x, y, A, B, C)
#     z = weight_A * A[2] + weight_B * B[2] + weight_C * C[2]
#     return z
#
# def calculate_normal(self, A, B, C):
#     """
#     Calculate the normal vector of a triangle defined by vertices A, B, and C.
#     Each vertex is a tuple (x, y, z).
#     """
#     # Convert vertices to numpy arrays
#     A = np.array(A)
#     B = np.array(B)
#     C = np.array(C)
#
#     # Calculate two vectors in the plane of the triangle
#     vector1 = B - A
#     vector2 = C - A
#
#     # Calculate the cross product, which is perpendicular to the plane of the triangle
#     normal = np.cross(vector1, vector2)
#
#     # Normalize the vector to have a length of 1
#     normal_unit = normal / np.linalg.norm(normal)
#
#     return normal_unit
#
# def axis_angle_to_euler(self, vertical_normal, target_normal):
#     """
#     Calculate the Euler angles to rotate 'vertical_normal' to 'target_normal'.
#     """
#     # Ensure normals are unit vectors
#     vertical_normal = vertical_normal / np.linalg.norm(vertical_normal)
#     target_normal = target_normal / np.linalg.norm(target_normal)
#
#     # Axis of rotation (cross product)
#     axis = np.cross(vertical_normal, target_normal)
#
#     # Angle of rotation (dot product and arccos)
#     angle = np.arccos(np.dot(vertical_normal, target_normal))
#     # print(axis,angle)
#     # Convert to quaternion
#     quaternion = self.axis_angle_to_quaternion(axis, angle)
#
#     # Convert to Euler angles
#     euler_angles = self.quaternion_to_euler(quaternion)
#     euler_deg = []
#     for rad in euler_angles:
#         euler_deg.append(math.degrees(rad))
#     # print("Euler Angles:", euler_deg)
#     return euler_deg
#
# def axis_angle_to_quaternion(self, axis, angle):
#     """
#     Convert an axis-angle rotation to a quaternion.
#     """
#     axis = axis / np.linalg.norm(axis)  # Normalize the axis
#     qx = axis[0] * np.sin(angle / 2)
#     qy = axis[1] * np.sin(angle / 2)
#     qz = axis[2] * np.sin(angle / 2)
#     qw = np.cos(angle / 2)
#     return np.array([qw, qx, qy, qz])
#
# def quaternion_to_euler(self, quaternion):
#     """
#     Convert a quaternion to Euler angles.
#     """
#     qw, qx, qy, qz = quaternion
#     # Roll (x-axis rotation)
#     sinr_cosp = 2 * (qw * qx + qy * qz)
#     cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)
#
#     # Pitch (y-axis rotation)
#     sinp = 2 * (qw * qy - qz * qx)
#     if np.abs(sinp) >= 1:  # Use 90 degrees if out of range
#         pitch = np.copysign(np.pi / 2, sinp)
#     else:
#         pitch = np.arcsin(sinp)
#
#     # Yaw (z-axis rotation)
#     siny_cosp = 2 * (qw * qz + qx * qy)
#     cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)
#
#     return np.array([roll, pitch, yaw])
