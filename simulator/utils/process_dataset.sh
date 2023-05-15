#!/bin/bash

# --help first arg is input second is output folder
if [ $1 == "--help" ]; then
    echo "Usage: $0 <input_folder> <output_folder>"
    exit 1
fi

# Check if number of args is two
if [ $# -ne 2 ]; then
    echo "Usage: $0 <input_folder> <output_folder>"
    exit 1
fi

# input path is the first argument
input_path=$1
# output path is the second argument
output_path=$2

# for all and only the folders in $input_path run filter_compress.sh
for folder in $input_path/*; do
    if [ -d $folder ]; then
        echo "Processing $folder"
        ./filter_compress.sh $folder $output_path
    fi
done
