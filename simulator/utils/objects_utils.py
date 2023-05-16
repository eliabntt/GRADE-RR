import utils.misc_utils
from utils.misc_utils import *

mtl_created_list = []

def setup_shapenet(username, password, csv_location):
	global database
	shapenet.settings.ShapenetSettings()
	if not os.path.exists(csv_location):
		logged_in = shapenet.login.save_v1_csvs(username, password, csv_location)
	database = shapenet.globals.get_database()
	return database


def load_object(rng=np.random.default_rng(), obj_name="shapenet", config=None, scale=1):
	if obj_name == "shapenet":
		return load_shapenet_object(rng, config, scale)
	elif obj_name == "google":
		return load_google_obj(rng, config, scale)


def load_shapenet_object(rng=np.random.default_rng(), config=None, scale=1):
	"""
	It loads a random object from the ShapeNet database

	:param rng: a random number generator. If you don't have one, you can use np.random.default_rng()
	:param config: a dictionary of parameters that can be set by the user
	:param scale: The scale of the object, defaults to 1 (optional)
	:return: The path to the object and the synsetId and modelId of the object.
	"""
	global database
	scale /= 100
	synsetId = rng.choice(list(database)) if config["synsetId"].get() == "random" else config["synsetId"].get()
	modelId = rng.choice(list(database[synsetId])) if config["modelId"].get() == "random" else config["modelId"].get()
	_settings = carb.settings.get_settings()
	prim = shapenet.shape.addShapePrim(_settings.get("/isaac/shapenet/omniverseServer"), synsetId, modelId,
	                                   Gf.Vec3d(0, 0, 0),
	                                   Gf.Rotation(Gf.Vec3d(1, 0, 0), 0),
	                                   scale, True, True)
	if type(prim) == str:
		raise Exception(prim)
	return str(prim.GetPath()), [synsetId, modelId]


def load_google_obj(rng=np.random.default_rng(), config=None, scale = 1):
	"""
	It loads a random Google 3D asset from the Google Scanned Object, converts it to USD, and then creates a reference to it
	in the current stage

	:param rng: a random number generator
	:param config: a dictionary of the config file
	:return: The prim path of the asset and the name of the asset
	"""
	google_obj_folder = config['google_obj_folder'].get()
	if config['google_obj_shortlist'].get() == "":
		asset = rng.choice(os.listdir(google_obj_folder))
	else:
		with (open(config['google_obj_shortlist'].get(), 'r')) as f:
			asset = rng.choice(f.read().splitlines())

	if not os.path.exists(f"{google_obj_folder}/exported_usd/{asset}/"):
		os.makedirs(f"{google_obj_folder}/exported_usd/{asset}/")
	usd_asset_path = f"{google_obj_folder}/exported_usd/{asset}/{asset}.usd"
	obj_asset_path = f"{google_obj_folder}/{asset}/meshes/model.obj"
	print(f"Converting {obj_asset_path} to {usd_asset_path}")
	if not os.path.exists(usd_asset_path):
		success = asyncio.new_event_loop().run_until_complete(convert_google_obj(obj_asset_path, usd_asset_path))
		if not success:
			raise Exception("Failed to convert obj to usd")
	stage = omni.usd.get_context().get_stage()
	prim_path = str(stage.GetDefaultPrim().GetPath()) + "/" + asset
	insta_count = 0
	prim_path_len = len(prim_path)
	while stage.GetPrimAtPath(prim_path):
		insta_count += 1
		prim_path = f"{prim_path[:prim_path_len]}_{insta_count}"

	omni.kit.commands.execute('CreateReferenceCommand',
	                          usd_context=omni.usd.get_context(),
	                          path_to=prim_path,
	                          asset_path=usd_asset_path,
	                          instanceable=True)
	texture_list = os.listdir(f"{google_obj_folder}/{asset}/materials/textures")[0]
	# shader = UsdShade.Shader(stage.GetPrimAtPath(f"{prim_path}/Looks/material_0/material_0"))
	# shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset)
	# omni.kit.commands.execute('ChangePropertyCommand',
	#                           prop_path=f'{prim_path}/Looks/material_0/material_0.inputs:diffuse_texture',
	#                           value=f"{google_obj_folder}/{asset}/materials/textures/{texture_list}",
	#                           prev=None)
	global mtl_created_list
	omni.kit.commands.execute(
		"CreateAndBindMdlMaterialFromLibrary",
		mdl_name="OmniPBR.mdl",
		mtl_name=f"{asset}",
		mtl_created_list=mtl_created_list,
	)
	mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
	omni.usd.create_material_input(
		mtl_prim,
		"diffuse_texture",
		"my-computer://" + texture_list, # my-computer seems necessary
		Sdf.ValueTypeNames.Asset,
	)
	obj_shade = UsdShade.Material(mtl_prim)
	for child in stage.GetPrimAtPath(prim_path).GetAllChildren():
		if child.GetTypeName().lower() == "xform":
			for subchild in child.GetAllChildren():
				if subchild.GetTypeName().lower() == "mesh":
					UsdShade.MaterialBindingAPI(subchild).Bind(obj_shade, UsdShade.Tokens.strongerThanDescendants)

	set_scale(stage.GetPrimAtPath(prim_path), scale)

	return str(prim_path), asset


async def convert_google_obj(in_path, out_path):
	"""
	It converts a Google 3D model to a format that can be used in Omni

	:param in_path: The path to the file you want to convert
	:param out_path: The path to the output file
	:return: A boolean value.
	"""
	import omni.kit.asset_converter as assetimport

	context = omni.kit.asset_converter.AssetConverterContext()
	converter_manager = omni.kit.asset_converter.get_instance()
	context.embed_textures = False

	task = converter_manager.create_converter_task(in_path, out_path, None, context)
	success = await task.wait_until_finished()
	return success


def load_objects(config, environment, rng, dynamic_prims, scale):
	"""
	Load objects in the environment

	Config should contain `config["obstacles"]` with the various considered keys.
	In our case those are shapenet and google(scanned_objects)
	In config we define the # of objects for each class.
	If the import fails the system tries to load it from another class.

	For now we do not generate positions that are collision free, so the objects will go through obstacles/humans/camera.

	config: the config dictionary
	environment: the environment object
	rng: the global rng
	dynamic_prims: the list of dynamic prims that will be used in the main thread
	"""
	stage = omni.usd.get_context().get_stage()
	shapenet_obs = config["obstacles"]["shapenet"].get()
	google_obs = config["obstacles"]["google"].get()
	num_obstacles = shapenet_obs + google_obs
	loc = ''
	google_obs_used = []
	shapenet_obs_used = []
	meters_per_unit = environment.meters_per_unit
	if (num_obstacles > 0):
		print("Loading obstacles..")
		for n in range(num_obstacles):
			print("Loading obstacle {}".format(n))
			# set random valid location, use "camera"
			x, y, z, yaw = position_object(environment, type=2)
			if google_obs > 0:
				ob_type = "google"
				google_obs -= 1
			else:
				ob_type = "shapenet"
				if loc == '':
					loc = shapenet.globals.get_local_shape_loc()
					print("Location is {}".format(loc))
					csv_location = loc + "/v1_csv/"
					database = setup_shapenet(config["shapenet_username"].get(), config["shapenet_password"].get(), csv_location)
					if database is None:
						print("Error loading database, resort to google")
						ob_type = "google"
				shapenet_obs -= 1
			try:
				my_shape, shape_infos = load_object(rng, ob_type, config, scale)
			except:
				print("Error loading object, try with the other type")
				try:
					my_shape, shape_infos = load_object(rng, "google" if ob_type == "shapenet" else "shapenet", config, scale)
				except:
					print("Error loading object, giving up")
					continue
			google_obs_used.append(shape_infos) if ob_type == "google" else shapenet_obs_used.append(shape_infos)
			print(f"{my_shape} loaded.. pose and adding animation")
			clear_properties(my_shape)
			add_translate_anim(my_shape, Gf.Vec3d(x[0] / meters_per_unit, y[0] / meters_per_unit, z[0] / meters_per_unit))
			add_rotation_anim(my_shape,
			                  Gf.Vec3d(rng.uniform(0, 2 * np.pi), rng.uniform(0, 2 * np.pi), rng.uniform(0, 2 * np.pi)))

			dynamic_prims.append(stage.GetPrimAtPath(my_shape))
			num_keys = rng.choice(range(1, config["experiment_length"].get()), rng.integers(1, 10)).astype(float)
			num_keys.sort()
			for key in num_keys:
				key *= 1
				x, y, z, yaw = position_object(environment, type=2)
				add_translate_anim(my_shape, Gf.Vec3d(x[0] / meters_per_unit, y[0] / meters_per_unit, z[0] / meters_per_unit),
				                   key)
				add_rotation_anim(my_shape, Gf.Vec3d(rng.uniform(0, 360), rng.uniform(0, 360), rng.uniform(0, 360)),
				                  key)
			if ob_type == "google":
				add_colliders(my_shape)
			add_semantics(stage.GetPrimAtPath(my_shape), ob_type)

		print("Loading obstacle complete")
	return google_obs_used, shapenet_obs_used
