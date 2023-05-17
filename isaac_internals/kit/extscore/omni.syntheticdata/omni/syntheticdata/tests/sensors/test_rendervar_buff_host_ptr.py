# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import unittest

import numpy as np
import ctypes
import omni.kit.test
from omni.kit.viewport.utility import get_active_viewport
from pxr import UsdGeom, UsdLux

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.syntheticdata as syn

from ..utils import add_semantics

# Test the SyntheticData following nodes :
# - SdPostRenderVarTextureToBuffer : node to convert a texture device rendervar into a buffer device rendervar
# - SdPostRenderVarToHost : node to readback a device rendervar into a host rendervar
# - SdRenderVarPtr : node to expose in the action graph, raw device / host pointers on the renderVars
#
# the tests consists in pulling the ptr data and comparing it with the data ouputed by :
# - SdRenderVarToRawArray
#
class TestRenderVarBuffHostPtr(omni.kit.test.AsyncTestCase):

    _tolerance = 1.1
    _outputs_ptr = ["outputs:dataPtr","outputs:width","outputs:height","outputs:bufferSize","outputs:format"]
    _outputs_arr = ["outputs:data","outputs:width","outputs:height","outputs:bufferSize","outputs:format"]

    @staticmethod
    def _assert_equal_tex_infos(out_a, out_b):
        assert((out_a["outputs:width"] == out_b["outputs:width"]) and (out_a["outputs:height"] == out_b["outputs:height"]) and (out_a["outputs:format"] == out_b["outputs:format"]))

    @staticmethod
    def _assert_equal_buff_infos(out_a, out_b):
        assert((out_a["outputs:bufferSize"] == out_b["outputs:bufferSize"]))

    @staticmethod
    def _assert_equal_data(data_a, data_b):
        assert(np.amax(np.square(data_a - data_b)) < TestRenderVarBuffHostPtr._tolerance)

    def _get_raw_array(self, rv):
        return syn.SyntheticData.Get().get_node_attributes(rv + "ExportRawArray", TestRenderVarBuffHostPtr._outputs_arr, self.render_product)
        
    def _get_ptr_array(self, rv, ptr_suffix):
        ptr_outputs = syn.SyntheticData.Get().get_node_attributes(rv + ptr_suffix, TestRenderVarBuffHostPtr._outputs_ptr, self.render_product)
        c_ptr = ctypes.cast(ptr_outputs["outputs:dataPtr"],ctypes.POINTER(ctypes.c_ubyte))
        ptr_outputs["outputs:dataPtr"] = np.ctypeslib.as_array(c_ptr,shape=(ptr_outputs["outputs:bufferSize"],))
        return ptr_outputs

    def _assert_equal_rv_ptr(self, rv:str, ptr_suffix:str, texture=None):
        arr_out = self._get_raw_array(rv)
        ptr_out = self._get_ptr_array(rv,ptr_suffix)
        if not texture is None:
            if texture:
                TestRenderVarBuffHostPtr._assert_equal_tex_infos(arr_out,ptr_out)
            else:
                TestRenderVarBuffHostPtr._assert_equal_buff_infos(arr_out,ptr_out)
        TestRenderVarBuffHostPtr._assert_equal_data(arr_out["outputs:data"],ptr_out["outputs:dataPtr"])

    def _assert_equal_rv_arr(self, rv:str, ptr_suffix:str, texture=None):
        arr_out_a = self._get_raw_array(rv)
        arr_out_b = self._get_raw_array(rv+ptr_suffix)
        if not texture is None:
            if texture:
                TestRenderVarBuffHostPtr._assert_equal_tex_infos(arr_out_a,arr_out_b)
            else:
                TestRenderVarBuffHostPtr._assert_equal_buff_infos(arr_out_a,arr_out_b)
        TestRenderVarBuffHostPtr._assert_equal_data(arr_out_a["outputs:data"],arr_out_b["outputs:data"])
        

    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        world_prim = UsdGeom.Xform.Define(stage,"/World")
        UsdGeom.Xformable(world_prim).AddTranslateOp().Set((0, 0, 0))
        UsdGeom.Xformable(world_prim).AddRotateXYZOp().Set((0, 0, 0))

        sphere_prim = stage.DefinePrim("/World/Sphere", "Sphere")
        add_semantics(sphere_prim, "sphere")
        UsdGeom.Xformable(sphere_prim).AddTranslateOp().Set((0, 0, 0))
        UsdGeom.Xformable(sphere_prim).AddScaleOp().Set((77, 77, 77))
        UsdGeom.Xformable(sphere_prim).AddRotateXYZOp().Set((-90, 0, 0))
        sphere_prim.GetAttribute("primvars:displayColor").Set([(1, 0.3, 1)])

        capsule0_prim = stage.DefinePrim("/World/Sphere/Capsule0", "Capsule")
        add_semantics(capsule0_prim, "capsule0")
        UsdGeom.Xformable(capsule0_prim).AddTranslateOp().Set((3, 0, 0))
        UsdGeom.Xformable(capsule0_prim).AddRotateXYZOp().Set((0, 0, 0))
        capsule0_prim.GetAttribute("primvars:displayColor").Set([(0.3, 1, 0)])

        capsule1_prim = stage.DefinePrim("/World/Sphere/Capsule1", "Capsule")
        add_semantics(capsule1_prim, "capsule1")
        UsdGeom.Xformable(capsule1_prim).AddTranslateOp().Set((-3, 0, 0))
        UsdGeom.Xformable(capsule1_prim).AddRotateXYZOp().Set((0, 0, 0))
        capsule1_prim.GetAttribute("primvars:displayColor").Set([(0, 1, 0.3)])

        capsule2_prim = stage.DefinePrim("/World/Sphere/Capsule2", "Capsule")
        add_semantics(capsule2_prim, "capsule2")
        UsdGeom.Xformable(capsule2_prim).AddTranslateOp().Set((0, 3, 0))
        UsdGeom.Xformable(capsule2_prim).AddRotateXYZOp().Set((0, 0, 0))
        capsule2_prim.GetAttribute("primvars:displayColor").Set([(0.7, 0.1, 0.4)])

        capsule3_prim = stage.DefinePrim("/World/Sphere/Capsule3", "Capsule")
        add_semantics(capsule3_prim, "capsule3")
        UsdGeom.Xformable(capsule3_prim).AddTranslateOp().Set((0, -3, 0))
        UsdGeom.Xformable(capsule3_prim).AddRotateXYZOp().Set((0, 0, 0))
        capsule3_prim.GetAttribute("primvars:displayColor").Set([(0.1, 0.7, 0.4)])

        spherelight = UsdLux.SphereLight.Define(stage, "/SphereLight")
        spherelight.GetIntensityAttr().Set(30000)
        spherelight.GetRadiusAttr().Set(30)

        self.viewport = get_active_viewport()    
        self.render_product = self.viewport.render_product_path
        
    async def test_host_arr(self):
        render_vars = [
            "BoundingBox2DLooseSD",
            "SemanticLocalTransformSD"
        ]
        for rv in render_vars:
            syn.SyntheticData.Get().activate_node_template(rv + "ExportRawArray", 0, [self.render_product])
            syn.SyntheticData.Get().activate_node_template(rv + "hostExportRawArray", 0, [self.render_product])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        await omni.kit.app.get_app().next_update_async()
        for rv in render_vars:
            self._assert_equal_rv_arr(rv,"host", False)
    
    async def test_buff_arr(self):
        render_vars = [
            "Camera3dPositionSD",
            "DistanceToImagePlaneSD",
        ]
        for rv in render_vars:
            syn.SyntheticData.Get().activate_node_template(rv + "ExportRawArray", 0, [self.render_product])
            syn.SyntheticData.Get().activate_node_template(rv + "buffExportRawArray", 0, [self.render_product])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        await omni.kit.app.get_app().next_update_async()
        for rv in render_vars:
            self._assert_equal_rv_arr(rv, "buff")
    
    async def test_host_ptr(self):
        render_vars = [
            "BoundingBox2DTightSD",
            "BoundingBox3DSD",
            "InstanceMapSD"
        ]
        for rv in render_vars:
            syn.SyntheticData.Get().activate_node_template(rv + "ExportRawArray", 0, [self.render_product])
            syn.SyntheticData.Get().activate_node_template(rv + "hostPtr", 0, [self.render_product])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        await omni.kit.app.get_app().next_update_async()
        for rv in render_vars:
            self._assert_equal_rv_ptr(rv,"hostPtr",False)

    async def test_host_ptr_tex(self):
        render_vars = [
            "NormalSD",
            "DistanceToCameraSD"
        ]
        for rv in render_vars:
            syn.SyntheticData.Get().activate_node_template(rv + "ExportRawArray", 0, [self.render_product])
            syn.SyntheticData.Get().activate_node_template(rv + "hostPtr", 0, [self.render_product])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        await omni.kit.app.get_app().next_update_async()
        for rv in render_vars:
            self._assert_equal_rv_ptr(rv,"hostPtr",True)
    
    async def test_buff_host_ptr(self):
        render_vars = [
            "LdrColorSD",
            "InstanceSegmentationSD",
        ]
        for rv in render_vars:
            syn.SyntheticData.Get().activate_node_template(rv + "ExportRawArray", 0, [self.render_product])
            syn.SyntheticData.Get().activate_node_template(rv + "buffhostPtr", 0, [self.render_product])
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        await syn.sensors.next_sensor_data_async(self.viewport,True)
        await syn.sensors.next_sensor_data_async(self.viewport,True)        
        await omni.kit.app.get_app().next_update_async()
        for rv in render_vars:
            self._assert_equal_rv_ptr(rv, "buffhostPtr",True)

    # After running each test
    async def tearDown(self):
        pass
