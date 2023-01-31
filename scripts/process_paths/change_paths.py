import argparse
import confuse
import os


def change_path(c_line, prefix, my_cc_path, match_str, normpath):
	path = os.path.join(my_cc_path + c_line[c_line.find(match_str) + len(match_str):])
	if normpath:
		path = os.path.normpath(path[:-2]) + "@\n"
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

filename = "tmp/tmp.usda"

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

prefix = config["prefix"].get()
my_cc_path = config["cc_path"].get()
my_front_path = config["front3d_path"].get()
my_cloth_path = config["cloth3d_path"].get()
my_surr_path = config["surreal_path"].get()
normpath = config["normpath"].get()
with open(out_file_path, "w") as o_file, open(filename, "r") as i_file:
	lines = i_file.readlines()
	for line in lines:
		c_line = line
		if "cc_textures" in line:
			c_line = change_path(c_line, prefix, my_cc_path, "cc_textures", normpath)
		elif "3DFRONT" in line:
			c_line = change_path(c_line, prefix, my_cc_path, "3DFRONT", normpath)
		elif "cloth3d" in line:
			c_line = change_path(c_line, prefix, my_cc_path, "cloth_3d", normpath)
		elif "surreal" in line:
			c_line = change_path(c_line, prefix, my_cc_path, "surreal", normpath)
		o_file.write(c_line)
