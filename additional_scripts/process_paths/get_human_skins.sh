#!/usr/bin/env bash
set -e
# get input main folder
input_main_folder=$1
out_main_folder=$2

# set output folder as Desktop
output_folder=/home/ebonetto/Desktop/output

# expand PATH and PYTHONPATH with
PATH=$PATH:/media/ebonetto/WindowsData/USD/install/bin
PYTHONPATH=$PYTHONPATH:/media/ebonetto/WindowsData/USD/install/lib/python

#if out_main_folder does not exist, create it
if [ ! -d "$out_main_folder" ]; then
  mkdir -p $out_main_folder
fi

# for each folder in input_main_folder echo the name
for folder in $input_main_folder/* ; do
  echo $folder
  # get the folder name
  folder_name=$(basename $folder)
  # run usdcat on the usd file with the same name as the folder
  usdcat $folder/$folder_name.usd -o $output_folder/$folder_name.usda
  # remove _with_cache from folder_name
  new_folder_name=${folder_name/_with_cache/}
  echo $new_folder_name
  mapping_folder=$2/$new_folder_name
  # if mapping_folder does not exist, create it
  if [ ! -d "$mapping_folder" ]; then
    mkdir -p $mapping_folder
  fi
  cat $output_folder/$folder_name.usda | grep grey | head -n 1 | cut -d '.' -f 1 | rev | cut -d '\' -f 1 | rev > $mapping_folder/$new_folder_name.txt
  # rm the usda file
  rm $output_folder/$folder_name.usda
done
