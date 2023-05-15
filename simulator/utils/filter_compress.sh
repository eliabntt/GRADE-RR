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
output_path=$2

# create the output directory if it doesn't exist
mkdir -p $output_path

# copy Viewport0_occluded from the input directory to the output directory
# cp $input_path/Viewport0_occluded $output_path/Viewport0_occluded -r

# copy all npy, usd, png, yaml, log files from the input directory to the output directory
find $input_path -maxdepth 1 -type f -name "*.npy" -exec cp {} $output_path \;
find $input_path -maxdepth 1 -type f -name "*.usd" -exec cp {} $output_path \;
find $input_path -maxdepth 1 -type f -name "*.png" -exec cp {} $output_path \;
find $input_path -maxdepth 1 -type f -name "*.yaml" -exec cp {} $output_path \;
find $input_path -maxdepth 1 -type f -name "*.log" -exec cp {} $output_path \;

# copy the folder whose name does not starts with Viewport0 from the input directory to the output directory
find $input_path -maxdepth 1 -type d ! -type l -not -name "Viewport0*" ! -path $input_path -exec cp {} $output_path -r \;

# copy all the .bag* files from the input directory to the output directory
find $input_path -maxdepth 1 -type f -name "*.bag*" -exec cp {} $output_path \;

# for every *.bag* file, if .active is in the name, run rosbag reindex, and remove the .active file
for file in $output_path/*.bag*; do
    if [[ $file == *.active ]]
    then
        echo "Reindexing $file"
        # get substring of $file without the .active extension
        file_new=${file%.active}
        mv $file $file_new
        rosbag reindex $file_new
        rm ${file_new%.bag}.orig.bag
    fi
done

# for each .bag file in the output directory, run rosbag reindex on it, and delete the .bag.orig file
for bag in $output_path/*.bag; do
    echo "Compressing $bag"
    # get the file name
    file_name=$(basename $bag)
    # move $bag to the same directory with the same name but with old_ prepended
    mv $bag $output_path/old_$file_name
    # bag is now the new filename
    bag=$output_path/old_$file_name
    # newbag is the old filename
    newbag=$output_path/$file_name

    rosbag filter $bag $newbag "'joint' in topic or \
                                 'tf' in topic or \
                                 'imu' in topic or \
                                 'odom' in topic or \
                                 'pose' in topic or \
                                 'camera_link/1' in topic or \
                                 'clock' in topic or \
                                 'command' in topic or \
                                 'exploration_node' in topic or \
                                 'predicted_state' in topic or \
                                 'reference_trajectory' in topic"

    rm $bag
    rosbag compress $newbag
    rm ${newbag%.bag}.orig.bag
done