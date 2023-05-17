import random

import numpy as np
from pxr import Gf, Semantics


def add_semantics(prim, semantic_label, semantic_type="class"):
    if not prim.HasAPI(Semantics.SemanticsAPI):
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set(semantic_type)
        sem.GetSemanticDataAttr().Set(semantic_label)


def get_random_transform():
    camera_tf = np.eye(4)
    camera_tf[:3, :3] = Gf.Matrix3d(Gf.Rotation(np.random.rand(3).tolist(), np.random.rand(3).tolist()))
    camera_tf[3, :3] = np.random.rand(3).tolist()
    return Gf.Matrix4d(camera_tf)
