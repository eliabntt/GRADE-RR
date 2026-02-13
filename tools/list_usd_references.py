from pxr import Usd

stage = Usd.Stage.Open('GRADE-RR/usds/env_base.usd')
print('Sublayers:', stage.GetRootLayer().subLayerPaths)
print('References:')
for prim in stage.Traverse():
    refs = prim.GetReferences()
    # refs.references is a list of SdfReference objects
    if hasattr(refs, 'references'):
        for ref in refs.references:
            if hasattr(ref, 'assetPath') and ref.assetPath:
                print(f"Prim: {prim.GetPath()} Asset Reference: {ref.assetPath}")
