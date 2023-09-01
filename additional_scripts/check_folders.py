"""
Use this to check if all the files/folders are there
"""
import os
import ipdb

mainpath = "/ps/project/irotate/"
folders = ["DE_lot_obs_cam0"]
tocheck = ["bbox_2d_loose","bbox_2d_tight","bbox_3d","camera","depthLinear","instance","poses","rgb"]

for mainfolder in folders:
	for folder in os.listdir(os.path.join(mainpath, mainfolder)):
		for subfolder in [os.path.join(mainpath, mainfolder, folder, "Viewport0"), os.path.join(mainpath, mainfolder, folder, "Viewport0_occluded")]:
			print(subfolder)
			data = os.listdir(subfolder)
			if len(data) > len(tocheck):
				print("More than expected folders")
				print(subfolder)
				ipdb.set_trace()
			if len(data) < len(tocheck):
				print("Less than expected folders")
				print(subfolder)
				ipdb.set_trace()
			for f in data:
				if f not in tocheck:
					continue
				if len(os.listdir(os.path.join(subfolder, f))) != 1801:
					print("Not enough files in folder")
					print(os.path.join(subfolder, f))
					ipdb.set_trace()
					

