#!/bin/zsh
### Code used to correct all the data in the folders
### This include the bboxes, the poses
### The code will also save the humans skeletal and vertices (decimated) information
set -e

# set input folder from arguments
input_folder=$1

cd /media/ebonetto/WindowsData/ov/isaac_sim-2021.2.1/
source ~/.zshrc
folder_isaac=/media/ebonetto/WindowsData/ov/isaac_sim-2021.2.1/

# for each folder in input_folder run correct.sh
for folder in $input_folder/* ; do

    echo "Correcting folder: $folder"

    screen -L -Logfile ./isaaclog.log -d -m -S ISAACSIM zsh -i -c "cd ${folder_isaac} && ./python.sh GRADE-RR/simulator/correct_data.py \
    --experiment_folder=${folder} --output_dir_humans=${folder}/Viewport0_occluded/humans --headless=True \
    --output_dir_poses=${folder}/Viewport0_occluded/poses --slow=False --fast=False --both=True --decimate=100 --old_poses=${folder}/Viewport0_occluded/poses/1.npy"
    while screen -list | grep -q ISAACSIM
    do
        sleep 1
    done
done
