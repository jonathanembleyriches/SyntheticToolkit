import unittest

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless":True})
import SyntheticToolkit.utils.omni_utils  as utils

from pxr import Gf, Sdf, Semantics, UsdGeom

class TestUtilConverterMethods(unittest.TestCase):


    ### vector tests
    def test_array_to_gfvec3d(self):
        in_data = [1.0,1.0,1.0]
        out_type = Gf.Vec3d
        out_data = utils.convert_vector_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")

    def test_array_to_gfvec3f(self):
        in_data = [1.0,1.0,1.0]
        out_type = Gf.Vec3f
        out_data = utils.convert_vector_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")


    def test_gfvec3d_to_array(self):
        in_data = [1.0,1.0,1.0]
        out_type = Gf.Vec3d
        out_data = utils.convert_vector_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")

    def test_gfvec3f_to_array(self):
        in_data = Gf.Vec3f(1.0,1.0,1.0)
        out_type = list
        out_data = utils.convert_vector_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")

    def test_gfvec3d_to_array(self):
        in_data = Gf.Vec3d(1.0,1.0,1.0)
        out_type = list
        out_data = utils.convert_vector_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")


    #quat tests
    def test_array_to_gfquatf(self):
        in_data = [1.0,1.0,1.0,1.0]
        out_type = Gf.Quatf
        out_data = utils.convert_quaternion_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")

    def test_gfquatf_to_array(self):
        in_data = Gf.Quatf(1.0,1.0,1.0,1.0)
        out_type = list
        out_data = utils.convert_quaternion_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")

    def test_array_to_gfquatd(self):
        in_data = [1.0,1.0,1.0,1.0]
        out_type = Gf.Quatd
        out_data = utils.convert_quaternion_types(in_data, out_type )
        self.assertIsInstance(out_data, out_type, "Data is of correct type")


if __name__ == '__main__':
    unittest.main()

