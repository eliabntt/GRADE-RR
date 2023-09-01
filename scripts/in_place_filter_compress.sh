# get the inputdir
input_path=$1

for folder_base in $input_path; do
  # for folder in noisy_bags, reindex_bags
  for folder in $folder_base"/D" $folder_base"/DH" $folder_base"/WOH" $folder_base"/WO" $folder_base"/F" $folder_base"/FH" ; do
      echo $folder

      # set output_path as folder
      output_path=$folder

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
  done

  for folder in $folder_base"/S" $folder_base"/SH" ; do
      echo $folder

      # set output_path as folder
      output_path=$folder

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
                                       'camera_link/0' in topic or \
                                       'clock' in topic or \
                                       'command' in topic or \
                                       'exploration_node' in topic or \
                                       'predicted_state' in topic or \
                                       'reference_trajectory' in topic"

          rm $bag
          rosbag compress $newbag
          rm ${newbag%.bag}.orig.bag
      done
  done
done

