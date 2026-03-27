import os
from datetime import datetime

from isaacsim import SimulationApp


def log(msg: str):
    line = f"[{datetime.now().isoformat()}] smoke_test_isaac51: {msg}"
    print(line, flush=True)
    try:
        with open("/tmp/grade_rr_smoke_test.log", "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception:
        pass


app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import omni

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
base_env = os.path.join(repo_root, "usds", "empty.usd")
log(f"repo_root={repo_root}")
log(f"opening stage={base_env}")
omni.usd.get_context().open_stage(base_env, None)

for _ in range(4):
    app.update()

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("Stage is None after open_stage")

prim_count = sum(1 for _ in stage.Traverse())
log(f"stage_opened prim_count={prim_count}")

for _ in range(180):
    app.update()

log("smoke test completed")
app.close()
