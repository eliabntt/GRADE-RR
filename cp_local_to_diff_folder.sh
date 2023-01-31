dest_folder=$1

cp ./exts/omni.isaac.synthetic_recorder/omni/isaac/synthetic_recorder/extension_custom.py ../$dest_folder/exts/omni.isaac.synthetic_recorder/omni/isaac/synthetic_recorder/extension_custom.py
cp ./exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/writers/numpy.py ../$dest_folder/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/writers/numpy.py
cp ./exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/syntheticdata.py  ../$dest_folder/exts/omni.isaac.synthetic_utils/omni/isaac/synthetic_utils/syntheticdata.py
cp ./exts/omni.isaac.shapenet/omni/isaac/shapenet/shape.py ../$dest_folder/exts/omni.isaac.shapenet/omni/isaac/shapenet/shape.py
cp ./apps/* ../$dest_folder/apps
cp ./setup_python_env.sh ../$dest_folder/setup_python_env.sh
cp ./kill.sh ../$dest_folder/kill.sh
cp ./simulator/ $dest_folder/simulator
