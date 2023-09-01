import argparse
import confuse
import os


def change_path(c_line, prefix, my_cc_path, match_str, normpath, remove_prefix=True):
	if remove_prefix:
		offset = len(match_str)
	else:
		offset = -1
	path = os.path.join(my_cc_path + c_line[c_line.find(match_str) + offset:])

	if normpath:
		path = os.path.normpath(path[:path.rfind("@")].replace('\\',"/")) + path[path.rfind("@"):]
	new_path = c_line[:c_line.find("@") + 1] + prefix + path
	return new_path


parser = argparse.ArgumentParser(description="USD reference changer")
parser.add_argument("--config_file", type=str, default="parser_config.yaml")
parser.add_argument("--input", type=str)
parser.add_argument("--output_name", type=str, default="")
parser.add_argument("--output_dir", type=str, default="")
args, unknown = parser.parse_known_args()
config = confuse.Configuration("USDRefChanger", __name__)
config.set_file(args.config_file)
config.set_args(args)

filename = config["input"].get()

output_loc = config["output_dir"].get()
if output_loc == "":
	output_loc = os.path.dirname(config["input"].get())

out_name = config["output_name"].get()
if out_name == "":
	out_name = os.path.basename(config["input"].get())[:-4] + "_proc.usda"
else:
	if out_name[-4:] != "usda":
		out_name += ".usda"
out_file_path = os.path.join(output_loc, out_name)

prefix_cc = config["prefix_cc"].get()
my_cc_path = config["cc_path"].get()

prefix_3dfront = config["prefix_front3d"].get()
my_front_path = config["front3d_path"].get()

prefix_cloth3d = config["prefix_cloth3d"].get()
my_cloth_path = config["cloth3d_path"].get()

prefix_surreal = config["prefix_surreal"].get()
my_surr_path = config["surreal_path"].get()

normpath = config["normpath"].get()
with open(out_file_path, "w") as o_file, open(filename, "r") as i_file:
	lines = i_file.readlines()
	for line in lines:
		c_line = line
		if ".png" in line or ".jpg" in line or ".jpeg" in line or ".tga" in line or ".tif" in line or ".bmp" in line and "cc_textures" not in line:
			# remove 3D-FUTURE-model
			if "3D-FUTURE-model" in line:
				# import ipdb; ipdb.set_trace()
				c_line = line.replace("3D-FUTURE-model/", "")
			if "cc_textures" not in line: # and "../../" in line:
				# import ipdb; ipdb.set_trace()
				# add after ../../ 3D-FUTURE-model
				l_index = c_line.find("../../")
				c_line = c_line[:l_index+6] + "3D-FUTURE-model/" + c_line[l_index+6:]

		if "opacity_constant" in line or "reflection_roughness_constant" in line or "metallic_constant" in line:
			tmp = c_line.split(" ")
			tmp[-1] = tmp[-1].replace("\n", "")
			if "int" in tmp:
				tmp[tmp.index("int")] = "float"
			if float(tmp[-1]) == 0:
				tmp[-1] = str(0.00001)
			try:
				tmp[-1] = str(format(float(tmp[-1])))
			except:
				import ipdb; ipdb.set_trace()
			c_line = " ".join(tmp)+"\n"
		elif "cc_textures" in line:
			c_line = change_path(c_line, prefix_cc, my_cc_path, "cc_textures", normpath, remove_prefix=False)
		elif "3DFRONT" in line or "3D-FUTURE" in line:
			if "future" in line.lower():
				c_line = change_path(c_line, prefix_3dfront, my_front_path, "3D-FUTURE-model", normpath)
			else:
				import ipdb;

				ipdb.set_trace()
				c_line = change_path(c_line, prefix_3dfront, my_front_path, "3DFRONT", normpath)
		elif "cloth3d" in line:
			c_line = change_path(c_line, prefix_cloth3d, my_cloth_path, "cloth_3d", normpath)
		elif "surreal" in line:
			c_line = change_path(c_line, prefix_surreal, my_surr_path, "surreal", normpath)
		o_file.write(c_line)
