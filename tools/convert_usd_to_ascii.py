from pxr import Usd

usd_path = 'GRADE-RR/usds/env_base_flat.usd'
usda_path = 'GRADE-RR/usds/env_base_flat.usda'
stage = Usd.Stage.Open(usd_path)
stage.GetRootLayer().Export(usda_path)
print(f"Converted USD to ASCII: {usda_path}")
