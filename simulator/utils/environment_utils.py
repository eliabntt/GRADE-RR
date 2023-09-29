"""
Use this class to load the environment and the relative information.
The init function should be used to load the environment.
It will get the environment from a given folder and create the necessary support variables.
"""

from omni.isaac.occupancy_map import _occupancy_map
from omni.isaac.occupancy_map.scripts.utils import update_location, compute_coordinates, generate_image

import utils.misc_utils
from utils.misc_utils import *


class environment:
	def __init__(self, config, rng = np.random.default_rng(), local_file_prefix = '', meters_per_unit=0.01):
		self.get_environment(config, rng, local_file_prefix)
		self.meters_per_unit = meters_per_unit

	def set_meters_per_unit(self, meters_per_unit):
		self.meters_per_unit = meters_per_unit

	def get_environment(self, config, rng: np.random.default_rng, local_file_prefix: str):
		"""
		If the name is not specified the environment will be taken at random using the rng.
		Based on the config one can decide if
		1. loading the stl of the environment
		2. loading the environment limits with the npy file [note that this is preferable, otherwise default values will be used]
		3. Using the limits the system will compute the necessary translations to center the environment in 0,0,0

		config: the configuration processed by the main algorithm
		rng: global rng
		local_file_prefix: necessary to access the local storage from isaacsim
		"""
		self.env_usd_export_folder = config["env_path"].get()
		if config["fix_env"].get() != "":
			self.env_name = config["fix_env"].get()
		else:
			self.env_name = rng.choice([f for f in os.listdir(self.env_usd_export_folder) if not f.startswith('.')])
		self.env_path = local_file_prefix + os.path.join(self.env_usd_export_folder, self.env_name, self.env_name + ".usd")
		if config["use_stl"].get():
			self.env_stl_path = os.path.join(self.env_usd_export_folder, self.env_name, self.env_name + ".stl")
			self.env_mesh = trimesh.load(os.path.join(self.env_usd_export_folder, self.env_name, self.env_name + ".stl"))
		else:
			self.env_stl_path = None
			self.env_mesh = None
		if config["use_npy"].get():
			self.env_info = np.load(os.path.join(self.env_usd_export_folder, self.env_name, self.env_name + ".npy"),
			                        allow_pickle=True)
			self.env_info = self.env_info.tolist()
		else:
			self.env_info = [0, 0, 0, 0, 0, 0, np.array([[-1000, -1000], [-1000, 1000], [1000, 1000], [1000, -1000]])]
		self.env_limits = self.env_info[0:6]

		self.shifts = [(self.env_limits[0] + self.env_limits[3]) / 2, (self.env_limits[1] + self.env_limits[4]) / 2,
		               self.env_limits[2]]
		self.env_limits_shifted = [self.env_limits[i] - self.shifts[i % 3] for i, _ in enumerate(self.env_limits)]
		self.area_polygon = get_area(self.env_info[6])
		self.env_polygon = [Point(i[0], i[1], 0) for i in self.env_info[-1]]

	def generate_map(self, out_path: str, zlim=[0, 1], cell_size = 0.05, origin=[0, 0, 0]):
		"""
		WARNING: HACK! ALL UNKNWON ARE WHITE!
		Generates a map for the environment and save it to the out_path location in the disk.
		First it searches for a non colliding location.
		Then it creates a map of the environment.
		We ovverride the unknown color to be "white" (i.e. free) as the system map unknown unreachable areas.

		out_path: the folder where to save the map
		z_limit: height to consider for projection
		cell_size: size of a single cell in the map (cm)
		origin: computed origin. Must be a free cell
		"""

		bound = int(
			max(abs(self.env_limits_shifted[0]) + abs(self.env_limits_shifted[3]),
				abs(self.env_limits_shifted[1]) + abs(self.env_limits_shifted[4])) / self.meters_per_unit * 1.5)

		_om = _occupancy_map.acquire_occupancy_map_interface()

		lower_bound = [-bound, -bound, zlim[0]/ self.meters_per_unit]
		lower_bound = np.array(lower_bound) - np.array(origin) / self.meters_per_unit
		upper_bound = [bound, bound, zlim[1]/ self.meters_per_unit *.8]
		upper_bound = np.array(upper_bound) - np.array(origin) / self.meters_per_unit

		center = np.array(origin) / self.meters_per_unit
		center[2] += 0.1 / self.meters_per_unit  # 10 cm above the floor
		update_location(_om, center, lower_bound, upper_bound)
		_om.set_cell_size(cell_size/self.meters_per_unit)
		_om.generate()
		image_buffer = generate_image(_om, [0, 0, 0, 255], [255, 255, 255, 255], [255, 255, 255, 255])
		dims = _om.get_dimensions()
		_im = Image.frombytes("RGBA", (dims.x, dims.y), bytes(image_buffer))
		image_width = _im.width
		image_height = _im.height

		size = [0, 0, 0]
		size[0] = image_width * cell_size
		size[1] = image_height * cell_size
		scale_to_meters = 1.0 / self.meters_per_unit
		default_image_name = os.path.join(out_path, "map.png")

		top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(_om, cell_size)

		ros_yaml_file_text = "image: " + default_image_name
		ros_yaml_file_text += f"\nresolution: {float(cell_size / scale_to_meters)}"
		ros_yaml_file_text += (
			f"\norigin: [{float(bottom_left[0] / scale_to_meters)}, {float(bottom_left[1] / scale_to_meters)}, 0.0000]"
		)
		ros_yaml_file_text += "\nnegate: 0"
		ros_yaml_file_text += f"\noccupied_thresh: {0.65}"
		ros_yaml_file_text += "\nfree_thresh: 0.196"

		_im.save(default_image_name)

		with open(default_image_name[:-3] + "yaml", 'w') as f:
			f.write(ros_yaml_file_text)
		center = lower_bound
		center[2] = -100000000.0
		update_location(_om, center, [0, 0, 0], [0, 0, 0])
		_om.generate()

	# disable_extension('omni.isaac.occupancy_map')

	def load_and_center(self, prim_path: str = "/World/home", correct_paths_req: bool = False, push_in_floor: bool = False):
		"""
		Load the environment from the usd path env_path
		Center it wrt the world coordinate frames

		The environment is loaded at prim_path

		prim_path: path that the environment should have in the prim tree
		correct_paths_req: if True, corrects the paths of the assets in the environment
		push_in_floor: if True, pushes the environment in the floor a bit. Useful for thin meshes that sometimes are not correctly visualized (flickering)
		"""
		stage = omni.usd.get_context().get_stage()

		print("loading environment {}".format(self.env_name))
		# from omni.isaac.core.utils.nucleus import find_nucleus_server
		# result, nucleus_server = find_nucleus_server()
		res, _ = omni.kit.commands.execute('CreateReferenceCommand',
		                                   usd_context=omni.usd.get_context(),
		                                   path_to=prim_path,
		                                   asset_path=self.env_path,
		                                   # asset_path= nucleus_server + "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
		                                   instanceable=True)
		if res:
			clear_properties(prim_path)
			if correct_paths_req:
				print("Correcting paths... --- note that you might want to change utils/misc_utils.py:correct_paths")
				try:
					correct_paths(prim_path)
				except:
					print("Failed to correct paths for {}".format(prim_path))
					time.sleep(10)
			else:
				print("Not correcting paths --- check that all textures are visibile and the reflection maps are correct")
			# center the home in the middle of the environment
			set_translate(stage.GetPrimAtPath(prim_path), list(- np.array(self.shifts) / self.meters_per_unit))
			for child in stage.GetPrimAtPath(prim_path).GetAllChildren():
				if "xform" == child.GetTypeName().lower():
					clear_properties(str(child.GetPath()))
					if push_in_floor and "floor" not in str(child.GetPath()).lower():
						myold = child.GetProperty('xformOp:translate').Get()
						myold = [myold[0], myold[1], myold[2] - 0.04]
						set_translate(child, list(np.array(myold)))
			return prim_path
		else:
			raise Exception("Failed to load environment {}".format(self.env_name))
