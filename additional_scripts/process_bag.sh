#!/bin/bash
set -e
# get input main folder
input_main_folder=$1
# get the temp folder
temp_folder=$2

# if the temp folder does not exist create it
mkdir -p $temp_folder

shouldIprocess=false
# b0fe48b1-d6b1-4854-ba04-111d22289522 - tmp
# e2104869-2823-4736-9e92-bc25fd7c9502 - tmp2

# for each folder in input_main_folder echo the name
for folder in $input_main_folder/* ; do
  folder_name=$(basename $folder)
  # if folder name equal to ciao set shouldIprocess to true
  if [ $folder_name == "e3080420-c235-480d-8122-9ba120001e5e" ]; then
    shouldIprocess=true
  fi

  # if shouldIprocess is false continue
  if [ $shouldIprocess == false ]; then
    echo "not processing $folder_name"
    continue
  fi

  echo "processing $folder_name"

  # get the folder name
  folder_name=$(basename $folder)
  # create a folder in temp_folder with the folder name
  mkdir -p $temp_folder/$folder_name
  # copy the bag files to the temp folder
  cp $folder/*.bag* $temp_folder/$folder_name

  # run
  /media/ebonetto/WindowsData/GRADE_tools/preprocessing/process_data.sh -t bag -p $temp_folder/$folder_name
  /media/ebonetto/WindowsData/GRADE_tools/preprocessing/process_data.sh -t extract -p $temp_folder/$folder_name/reindex_bags

  # make a static_bag and a dynamic_bag folder in $folder
  mkdir -p $folder/static_bag
  mkdir -p $folder/dynamic_bag
  mkdir -p $temp_folder/$folder_name/static_bag
  mkdir -p $temp_folder/$folder_name/dynamic_bag

  reindex_folder=$temp_folder/$folder_name/reindex_bags
  for bag in $reindex_folder/*.bag; do
    file_name=$(basename $bag)

    newbag=$temp_folder/$folder_name/static_bag/$file_name
    rosbag filter $bag $newbag "'camera_link/1' not in topic"
    rosbag compress $newbag
    rm ${newbag%.bag}.orig.bag

    newbag=$temp_folder/$folder_name/dynamic_bag/$file_name
    rosbag filter $bag $newbag "'camera_link/0' not in topic"
    rosbag compress $newbag
    rm ${newbag%.bag}.orig.bag
  done

  # for each .bag in the $folder copy the .bag file with the same name from the $temp_folder/$folder_name
  for bag in $temp_folder/$folder_name/*.bag; do
    rosbag compress $bag
    rm ${bag%.bag}.orig.bag
    cp $bag $folder
  done

  # rm *.bag.active in $folder if they exist
  rm $folder/*.bag.active || true

  # mv all the static_bag and dynamic_bag to the $folder
  mv $temp_folder/$folder_name/static_bag/* $folder/static_bag
  mv $temp_folder/$folder_name/dynamic_bag/* $folder/dynamic_bag
  mv $temp_folder/$folder_name/reindex_bags/data $folder/exp_data

  # remove the temp folder
  rm -rf $temp_folder/$folder_name

done