if [ "$#" -ne 1 ]; then
     echo "nargs not valid. call ./req.sh main_isaac_folder"
     exit
fi
$1/python.sh -m pip install rtree pyquaternion ipdb rospkg confuse defusedxml mesh numpy-stl trimesh netifaces pyyaml pycryptodomex gnupg opencv-python
