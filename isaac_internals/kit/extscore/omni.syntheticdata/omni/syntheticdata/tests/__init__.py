"""
Presence of this file allows the tests directory to be imported as a module so that all of its contents
can be scanned to automatically add tests that are placed into this directory.
"""
scan_for_test_modules = True

from .sensors.test_bbox3d import *
from .sensors.test_bbox2d_loose import *
from .sensors.test_bbox2d_tight import *
from .sensors.test_distance_to_camera import *
from .sensors.test_distance_to_image_plane import *
from .sensors.test_depth import *  # *** DEPRECATED ***
from .sensors.test_depth_linear import *  # *** DEPRECATED ***
from .sensors.test_motion_vector import *
from .sensors.test_normals import *
from .sensors.test_occlusion import *
from .sensors.test_rgb import *
from .sensors.test_instance_seg import *
from .sensors.test_semantic_seg import *
from .sensors.test_cross_correspondence import *
from .sensors.test_swh_frame_number import *
from .sensors.test_renderproduct_camera import *
from .sensors.test_rendervar_buff_host_ptr import *
from .sensors.test_semantic_filter import *

from .helpers.test_instance_mapping import *
from .helpers.test_projection import *
from .helpers.test_bboxes import *

from .visualize.test_semantic_seg import *
from .visualize.test_flattener import *

from .pipeline.test_instance_mapping import *
from .pipeline.test_swh_frame_number import *
from .pipeline.test_renderproduct_camera import *

from .graph.test_graph_manipulation import *