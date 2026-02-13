from pxr import Usd

usd_path = 'GRADE-RR/usds/env_base_saved.usd'
flat_path = 'GRADE-RR/usds/env_base_flat.usd'
stage = Usd.Stage.Open(usd_path)
stage.GetRootLayer().Export(flat_path)
print(f"Flattened USD saved as {flat_path}")
