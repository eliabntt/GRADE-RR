#!/usr/bin/env python
"""Minimal Isaac Sim test to debug USD loading crashes."""

import os
import sys

# Add simulator directory to path
simulator_dir = os.path.dirname(os.path.abspath(__file__))
if simulator_dir not in sys.path:
    sys.path.insert(0, simulator_dir)

print("[TEST] Starting minimal Isaac Sim test...")

try:
    from isaacsim import SimulationApp
    print("[TEST] SimulationApp imported successfully")
    
    # Create SimulationApp
    CONFIG = {
        "display_options": 3286,
        "width": 1280,
        "height": 720,
        "headless": True,
    }
    
    print("[TEST] Creating SimulationApp...")
    app = SimulationApp(launch_config=CONFIG)
    print("[TEST] SimulationApp created successfully")
    
    import omni
    print("[TEST] omni module imported")
    
    # Try to open the stage without update
    stage_path = os.path.join(simulator_dir, "usds", "empty.usd")
    print(f"[TEST] Opening stage: {stage_path}")
    print(f"[TEST] Stage file exists: {os.path.exists(stage_path)}")
    
    if os.path.exists(stage_path):
        print("[TEST] Calling open_stage()...")
        omni.usd.get_context().open_stage(stage_path, None)
        print("[TEST] open_stage() completed, now trying update()...")
        
        try:
            app.update()
            print("[TEST] First app.update() succeeded")
            app.update()
            print("[TEST] Second app.update() succeeded")
        except Exception as e:
            print(f"[TEST] app.update() failed: {e}")
            import traceback
            traceback.print_exc()
    else:
        print(f"[TEST] Stage file not found: {stage_path}")
    
    print("[TEST] Test completed successfully")
    app.close()
    
except Exception as e:
    print(f"[TEST] Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
