# Minimal USD Loader for Asset Validation

import sys
from pxr import Usd

if len(sys.argv) != 2:
    print("Usage: python minimal_usd_loader.py <path_to_usd>")
    sys.exit(1)

usd_path = sys.argv[1]
print(f"[INFO] Attempting to open USD: {usd_path}")
try:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print("[ERROR] Failed to open stage (returned None)")
        sys.exit(2)
    print("[SUCCESS] USD loaded successfully.")
    print("Root prims:", [x.GetName() for x in stage.GetPseudoRoot().GetChildren()])
except Exception as e:
    print(f"[CRASH] Exception while loading USD: {e}")
    sys.exit(3)
