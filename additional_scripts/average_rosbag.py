"""
This is the code used to get the average acc speed and dynamic frames for the GRADE paper.
You need some experiment folders.
This code will use the bags files in those folder.
Please change the folders as desired (first loop in the code, first two lines).
We also suppose that you have the instance images to compute the percentage of dynamic frames.
"""

import rosbag
import sys
import numpy as np
import os

# loop through all the bags in the folder

folders = []
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/d94ecc9f-10f6-4f6d-b49f-1ed841f86772")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/d8c14dd6-d794-46d5-aa59-01d3552828c7")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/b13a4874-00a4-49a5-aa2d-e22d7d864b56")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/75bf66e8-acb0-4f27-842d-1945ad42f9de")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/53bfe530-122d-42cb-a1f4-453e6a2a617f")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/23aae785-c0bc-4645-9e64-fdea78c42e2d")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/b0a9c3c3-d470-45ea-82c6-ac529b6882ea")
folders.append("/ps/project/irotate/GRADE-paper/Test.bak2/12e463c1-4993-4ea8-9cbf-54ba9403e5f8")

names = ["d94ecc9f-10f6-4f6d-b49f-1ed841f86772","d8c14dd6-d794-46d5-aa59-01d3552828c7","b13a4874-00a4-49a5-aa2d-e22d7d864b56","75bf66e8-acb0-4f27-842d-1945ad42f9de","53bfe530-122d-42cb-a1f4-453e6a2a617f","23aae785-c0bc-4645-9e64-fdea78c42e2d","b0a9c3c3-d470-45ea-82c6-ac529b6882ea","12e463c1-4993-4ea8-9cbf-54ba9403e5f8"]
import pandas as pd
df = pd.DataFrame(columns=['name','speed','acc','dynamic_frames','dynamic_frames_avg_coverage'])

for folder in folders:
	bag_folder = os.path.join(folder, "reindex_bags")

	bags = []
	for bag in os.listdir(bag_folder):
		if bag.endswith(".bag"):
			for n in names:
				if n in bag:
					bags.append(bag)
					break
	# sort bags according to the number
	bags = sorted(bags, key=lambda x: int(x.split("_")[1].split(".")[0]))

	avg_speed = [] # avg absolute speed per axis
	avg_acc = [] # avg absolute acc per axis

	for bagname in bags:
		print(bagname)
		# open the bag
		bag = rosbag.Bag(os.path.join(bag_folder, bagname))
		old_t = None
		# loop through all the topics
		for topic, msg, t in bag.read_messages(topics=['/my_robot_0/odom']):
			# if the topic is the one we want
			if topic == "/my_robot_0/odom":
				# get the data
				data_lin = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
				data_ang = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

				# get the speed
				avg_speed.append([np.abs(data_lin[0]), np.abs(data_lin[1]), np.abs(data_lin[2]), np.abs(data_ang[0]), np.abs(data_ang[1]), np.abs(data_ang[2])])

				# get the acceleration by using the difference between the current and the previous time
				if old_t is None:
					old_speed = [data_lin[0], data_lin[1], data_lin[2], data_ang[0], data_ang[1], data_ang[2]]
					old_t = t
				else:
					# get the difference between the current and the previous time
					dt = (t - old_t).to_sec()
					# get the acceleration
					avg_acc.append(np.abs(np.array(
						[(data_lin[0] - old_speed[0]) / dt, (data_lin[1] - old_speed[1]) / dt, (data_lin[2] - old_speed[2]) / dt,
						 (data_ang[0] - old_speed[3]) / dt, (data_ang[1] - old_speed[4]) / dt, (data_ang[2] - old_speed[5]) / dt])))
					# update the old speed and time
					old_speed = [data_lin[0], data_lin[1], data_lin[2], data_ang[0], data_ang[1], data_ang[2]]
					old_t = t
		bag.close()
	df = pd.concat([df, pd.DataFrame([[bagname[:-6], np.round(np.mean(avg_speed, axis=0),3), np.round(np.mean(avg_acc, axis=0),3), 0, 0]],
	                            columns=df.columns)])

folders = []
folders.append("/ps/project/irotate/DE_few_obs_cam0_horiz/d94ecc9f-10f6-4f6d-b49f-1ed841f86772")
folders.append("/ps/project/irotate/DE_few_obs_cam0_horiz/d8c14dd6-d794-46d5-aa59-01d3552828c7")
folders.append("/ps/project/irotate/DE_cam0_horiz/b13a4874-00a4-49a5-aa2d-e22d7d864b56")
folders.append("/ps/project/irotate/DE_cam1/75bf66e8-acb0-4f27-842d-1945ad42f9de")
folders.append("/ps/project/irotate/DE_few_obs_cam1/53bfe530-122d-42cb-a1f4-453e6a2a617f")
folders.append("/ps/project/irotate/DE_lot_obs_cam0/23aae785-c0bc-4645-9e64-fdea78c42e2d")

import cv2

for folder in folders:
	dynamic_images = 0
	dynamic_coverage = 0

	masks = os.path.join(folder, "Viewport0_occluded/instance")
	for mask in os.listdir(masks):
		if mask.endswith(".npy"):
			f = np.load(os.path.join(masks, mask), allow_pickle=True)
			classes = []
			for item in f[1]:
				if item[3] == "human" or item[3] == "google" or item[3] == "shapenet":
					classes.append(item[0])
			""" opencv reshape f[0] to (640, 480) """
			img = cv2.resize(f[0].astype(np.uint16), (640, 480), interpolation=cv2.INTER_NEAREST)
			out = np.isin(img, classes)
			"""count the number of elements of img that are equal to an element of classes"""
			if len(out[out==True]) > 0:
				dynamic_coverage += len(out[out==True]) / img.size
				dynamic_images += 1
	df.loc[df["name"] == folder.split("/")[-1], "dynamic_frames"] = dynamic_images
	df.loc[df["name"] == folder.split("/")[-1], "dynamic_frames_avg_coverage"] = round(dynamic_coverage / dynamic_images*100,2)

# print dataframe as latex table
print(df.to_latex(index=False))

df.to_pickle("dynamic_frames.pkl")