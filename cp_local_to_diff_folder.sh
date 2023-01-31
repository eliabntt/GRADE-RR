if [ "$#" -ne 2 ]; then
    echo "nargs not valid. Call ./cp_local_to_diff_folder.sh source destination"
    exit
fi

src_folder=$1
dest_folder=$2

cp $src_folder/omni.isaac.synthetic_recorder/omni/isaac/synthetic_recorder/extension_custom.py $dest_folder/exts/omni.isaac.synthetic_recorder/omni/isaac/synthetic_recorder/extension_custom.py
cp $src_folder/isaac_internals/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/writers/numpy.py $dest_folder/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/writers/numpy.py
cp $src_folder/isaac_internals/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/syntheticdata.py  $dest_folder/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/syntheticdata.py
cp $src_folder/isaac_internals/exts/omni.isaac.shapenet/omni/isaac/shapenet/shape.py $dest_folder/exts/omni.isaac.shapenet/omni/isaac/shapenet/shape.py
cp $src_folder/isaac_internals/apps/* $dest_folder/apps
cp $src_folder/setup_python_env.sh $dest_folder/setup_python_env.sh
cp $src_folder/kill.sh $dest_folder/kill.sh
