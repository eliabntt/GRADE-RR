# Minimal Isaac Sim USD Loader (omni.usd)

import carb
import omni.usd
import sys

if len(sys.argv) != 2:
    print("Usage: python minimal_omni_usd_loader.py <path_to_usd>")
    sys.exit(1)

usd_path = sys.argv[1]
print(f"[INFO] Attempting to open USD with omni.usd: {usd_path}")
try:
    ctx = omni.usd.get_context()
    stage = ctx.open_stage(usd_path)
    if stage is None:
        print("[ERROR] omni.usd failed to open stage (returned None)")
        sys.exit(2)
    print("[SUCCESS] omni.usd loaded USD successfully.")
    print("Stage default prim:", stage.GetDefaultPrim().GetName())
except Exception as e:
    print(f"[CRASH] Exception while loading USD with omni.usd: {e}")
    sys.exit(3)
