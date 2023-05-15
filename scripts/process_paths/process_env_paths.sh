#!/usr/bin/env bash
set -e
# get input main folder
input_main_folder=$1

# set output folder as Desktop
output_folder=/home/ebonetto/Desktop/output

# expand PATH and PYTHONPATH with
PATH=$PATH:/media/ebonetto/WindowsData/USD/install/bin
PYTHONPATH=$PYTHONPATH:/media/ebonetto/WindowsData/USD/install/lib/python

# for each folder in input_main_folder echo the name
for folder in $input_main_folder/* ; do
  echo $folder
  # get the folder name
  folder_name=$(basename $folder)
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
