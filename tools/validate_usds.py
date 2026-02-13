# USD Asset Validator for GRADE-RR/usds/

import os
import sys
from pxr import Usd

usds_dir = sys.argv[1] if len(sys.argv) > 1 else "GRADE-RR/usds"

for fname in os.listdir(usds_dir):
    if not fname.endswith(".usd"):
        continue
    path = os.path.join(usds_dir, fname)
    print(f"\n[INFO] Validating: {path}")
    try:
        stage = Usd.Stage.Open(path)
        if stage is None:
            print("[ERROR] Failed to open stage (returned None)")
            continue
        print("[SUCCESS] USD loaded. Root prims:", [x.GetName() for x in stage.GetPseudoRoot().GetChildren()])
        # Print all prim types for inspection
        for prim in stage.Traverse():
            print(f"  Prim: {prim.GetName()}  Type: {prim.GetTypeName()}")
    except Exception as e:
        print(f"[CRASH] Exception while loading {fname}: {e}")
