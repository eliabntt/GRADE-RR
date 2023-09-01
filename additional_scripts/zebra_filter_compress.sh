#!/bin/bash
set -e

# --help first arg is input second is output folder
if [ $1 == "--help" ]; then
    echo "Usage: $0 <input_file> <output_file>"
    exit 1
fi

# Check if number of args is two
if [ $# -ne 2 ]; then
    echo "Usage: $0 <input_file> <output_file>"
    exit 1
fi

# input path is the first argument
input_path=$1
# output path is the second argument
output_main_path=$2

# loop through all the folders in the input directory
for folder in $input_path/*; do
  # create the output directory if it doesn't exist
  output_path=$output_main_path/$(basename $folder)
  mkdir -p $output_path

  # copy all npy, usd, png, yaml, log files from the input directory to the output directory
  find $folder -maxdepth 1 -type f -name "*.npy" -exec cp {} $output_path \;
  find $folder -maxdepth 1 -type f -name "*.usd" -exec cp {} $output_path \;
  find $folder -maxdepth 1 -type f -name "*.png" -exec cp {} $output_path \;
  find $folder -maxdepth 1 -type f -name "*.yaml" -exec cp {} $output_path \;
  find $folder -maxdepth 1 -type f -name "*.log" -exec cp {} $output_path \;

  # list all Vieport directories in $folder and associate that to the viewports variable
  viewports=$(find $folder -maxdepth 1 -type d -name "Viewport*")

  # for every viewport in viewports mkdir in $output_path
  for viewport in $viewports;
  do
      mkdir -p "$output_path/$(basename $viewport)/rgb"
      # for every file in rgb, if it is a .png file, copy it to the rgb directory while converting it to jpg
      mkdir -p "$output_path/$(basename $viewport)"
      # copy all the folders except for rgb
      find $viewport -maxdepth 1 -type d ! -type l -not -name "rgb" ! -path $viewport -exec cp {} "$output_path/$(basename $viewport)" -r \;

      # create the rgb directory
      for file in $viewport/rgb/*; do
          if [[ $file == *.png ]]
          then
              # get the file name
              file_name=$(basename $file)
              # convert the file to jpg
              convert $file "$output_path/$(basename $viewport)/rgb/${file_name%.png}.jpg"
          fi
      done
  done

  # compress the output directory into a tar.gz file and delete the output directory
  tar -czvf $output_path.tar.gz $output_path --remove-files
done