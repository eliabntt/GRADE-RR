#!/usr/bin/env bash
set -e
# get input main folder
input_main_folder=$1

# set output folder as Desktop
output_folder=/home/ebonetto/Desktop/output

# expand PATH and PYTHONPATH with
PATH=$PATH:/media/ebonetto/WindowsData/USD/install/bin
PYTHONPATH=$PYTHONPATH:/media/ebonetto/WindowsData/USD/install/lib/python

# set shouldIprocess to false
shouldIprocess=false

# for each folder in input_main_folder echo the name
for folder in $input_main_folder/* ; do
  echo $folder
  # get the folder name
  folder_name=$(basename $folder)

  # if folder_name == 3e40b128-6291-41ff-89aa-0ae707a594c6 set shouldIprocess to true
  if [ $folder_name == "36810ab3-d383-431d-9cda-f58c70c83c5e" ]; then
    shouldIprocess=true
  fi
  # if not shouldIprocess continue
  if [ $shouldIprocess == false ]; then
    echo "not processing $folder_name"
    continue
  fi

  # run usdcat on the usd file with the same name as the folder
  usdcat $folder/$folder_name.usd -o $output_folder/$folder_name.usda
  # run the python script to change the paths
  python3 ./change_paths.py --input $output_folder/$folder_name.usda --output_dir $output_folder --output_name ${folder_name}_proc
  # run usdcat on the usda file to convert it to usd
  usdcat $output_folder/${folder_name}_proc.usda -o $output_folder/$folder_name.usd
  # remove the usda files
  rm $output_folder/$folder_name.usda
  rm $output_folder/${folder_name}_proc.usda
  # mv the new usd file to the input folder
  mv $output_folder/$folder_name.usd $folder/$folder_name.usd
done
