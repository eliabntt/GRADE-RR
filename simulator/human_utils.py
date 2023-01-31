import utils
from utils import *


def move_humans_to_ground(my_humans_heights: list, body_lists: list, frame: float, meters_per_unit: float,
                          max_height: float):
    """
    Function to keep the human at ground level (0 for now, but can be elaborated)

    my_human_heights: list of [animation_frames, [vertices, z_loc]]. For every frame of the animation, for every vertex, the z loc
    body_lists: Using to access the prim, list of prim paths
    frame: the simulation frame we are in (float or int will get a cast to int)
    meters_per_unit: meter per unit of distance in the simulation
    """
    stage = omni.usd.get_context().get_stage()
    for index, height in enumerate(my_humans_heights):
        z_min = None
        if height is None:
            context = omni.usd.get_context()
            stage = context.get_stage()
            prim = stage.GetPrimAtPath(body_lists[index])
            for i in prim.GetAllChildren():
                if "armature" in str(i.GetPath()).lower():
                    prim = i
            for i in prim.GetAllChildren():
                if "body" in str(i.GetPath()).lower():
                    prim = i
            for i in prim.GetAllChildren():
                if "mesh" in str(i.GetPath()).lower():
                    prim = i
            l = prim.GetPropertyNames()
            if "points" in l:
                k = prim.GetAttribute("points").Get()
                if k is not None:
                    k = np.array(k)
                    z_min = min(k[:, 2])
        else:
            z_min = min(height[int(min(max(frame - 1, 0), len(height) - 1))]) / meters_per_unit
        if z_min is None:
            continue
        if z_min < max_height:
            loc = stage.GetPrimAtPath(body_lists[index]).GetProperty('xformOp:translate').Get()
            loc = [loc[0], loc[1], loc[2] - z_min]
            set_translate(stage.GetPrimAtPath(body_lists[index]), loc)


def load_human(human_base_prim_path, n, asset_path, dynamic_prims, added_prims):
    """
    Load the human based on the usd path and add it to the dynamic prims list
    Follow prim naming convention /human_base_prim_path+n
    Add also the semantic with the label "human"

    human_base_prim_path: the base path to which we add the n of the n-th human as per the prim path
    n: the number of the human
    asset_path: the path of the ussd of the human
    dynamic_prims: the list of dynamic prims in the world. Only the body, and the clothes will be added (not the armature) as separate objects
    """
    stage = omni.usd.get_context().get_stage()
    res, _ = omni.kit.commands.execute("CreateReferenceCommand",
                                       usd_context=omni.usd.get_context(),
                                       path_to=f"{human_base_prim_path}{n}",
                                       asset_path=asset_path,
                                       instanceable=False)
    cnt = 0
    if res:
        for child in stage.GetPrimAtPath(f"{human_base_prim_path}{n}").GetAllChildren():
            if "armature" in child.GetName().lower():
                for sub_child in child.GetAllChildren():
                    if "armature" not in sub_child.GetName().lower():
                        dynamic_prims.append(sub_child)
                        cnt += 1
        added_prims.append(cnt)
        clear_properties(f"{human_base_prim_path}{n}")
        #correct_paths(f"{human_base_prim_path}{n}", "human")
    else:
        raise Exception(f"Failed to load human {n} from {asset_path}")


# todo enable?
def create_human_message(human_prim, skel_prim, index, meters_per_unit, body_origins):
    stage = omni.usd.get_context().get_stage()

    skeleton, joint_token = AnimationSchema.SkelJoint(skel_prim).GetJoint()
    time_code = omni.timeline.get_timeline_interface().get_current_time() * stage.GetTimeCodesPerSecond()
    skel_cache = UsdSkel.Cache()
    skel_query = skel_cache.GetSkelQuery(UsdSkel.Skeleton(skeleton.GetPrim()))
    transforms = skel_query.ComputeJointLocalTransforms(time_code)
    translates, rotations, scales = UsdSkel.DecomposeTransforms(transforms)
    joint_order = skel_query.GetJointOrder()

    ## todo build message
    pass


# todo enable?
def get_skeleton_info(meters_per_unit, body_origins, human_prim_list):
    """
    Get the global position of the skeleton joints.
    Unused.
    """
    # fixme account for vertical shift if needed
    stage = omni.usd.get_context().get_stage()
    for index, handle in enumerate(body_lists):
        human_prim = stage.GetPrimAtPath(handle)
        for p in human_prim.GetAllChildren():
            if "armature" in str(p.GetPrim()).lower():
                for pp in p.GetAllChildren():
                    if "armature" in str(pp.GetPrim()).lower().split("/")[-1]:
                        for ppp in pp.GetAllChildren():
                            if "root" in str(ppp.GetPrim()).lower():
                                create_human_message(human_prim, ppp, index, meters_per_unit, body_origins[index])
