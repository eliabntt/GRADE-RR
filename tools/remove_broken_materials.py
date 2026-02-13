from pxr import Usd, UsdShade

usd_path = 'GRADE-RR/usds/env_base.usd'
stage = Usd.Stage.Open(usd_path)

# List all prims with materials referencing omniverse://
def remove_broken_materials(stage):
    for prim in stage.Traverse():
        mat = UsdShade.Material(prim)
        if mat:
            shader_attr = mat.GetSurfaceAttr()
            if shader_attr:
                val = shader_attr.Get()
                if isinstance(val, str) and 'omniverse://' in val:
                    print(f"Removing broken material from {prim.GetPath()}")
                    shader_attr.Set('')

remove_broken_materials(stage)
stage.GetRootLayer().Save()
print("Broken omniverse:// material references removed and USD saved.")
