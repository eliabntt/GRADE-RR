# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from .global_constants import *
import carb.tokens
import os
import bz2
import omni

try:
    import cPickle as pickle
except:
    import pickle

DEBUG_PRINT_ON = False

g_local_shape_loc = None

g_shapenet_db = None

g_pickle_file_full_name = None


def pickle_file_exists():
    global g_pickle_file_full_name
    if g_pickle_file_full_name == None:
        g_pickle_file_full_name = os.path.realpath(__file__)[:-11] + g_pickle_file_name
    return os.path.exists(g_pickle_file_full_name)


def get_database():
    global g_shapenet_db
    if g_shapenet_db == None:
        if pickle_file_exists():
            f = bz2.BZ2File(g_pickle_file_full_name, "rb")
            g_shapenet_db = pickle.load(f)
            f.close()
        else:
            g_shapenet_db = None
            omni.kit.app.get_app().print_and_log(f"Missing shapenet database of names at {g_pickle_file_full_name}.")
    return g_shapenet_db


def get_local_shape_loc():
    global g_local_shape_loc
    if g_local_shape_loc == None:

        env_path = os.getenv("SHAPENET_LOCAL_DIR")
        if env_path is None:
            resolved_data_path = carb.tokens.get_tokens_interface().resolve("${data}")
            g_local_shape_loc = resolved_data_path + "/shapenet"
            omni.kit.app.get_app().print_and_log(
                f"env var SHAPENET_LOCAL_DIR not set, using default data dir {g_local_shape_loc}"
            )
        else:
            g_local_shape_loc = env_path
            omni.kit.app.get_app().print_and_log(f"Using local env var SHAPENET_LOCAL_DIR {env_path}")

    return g_local_shape_loc
