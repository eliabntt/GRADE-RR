#!/bin/bash
# source ~/catkin_ws/devel/setup.bash
#source ~/catkin_ws/devel/setup.bash --extend
SCRIPT_DIR="$(dirname "${BASH_SOURCE}")"
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/../../../$PYTHONPATH:$SCRIPT_DIR/exts/omni.isaac.kit:$SCRIPT_DIR/exts/omni.isaac.gym:$SCRIPT_DIR/kit/kernel/py:$SCRIPT_DIR/kit/plugins/bindings-python:$SCRIPT_DIR/kit/extscore/omni.kit.pip_archive/pip_prebundle:$SCRIPT_DIR/exts/omni.isaac.core_archive/pip_prebundle:$SCRIPT_DIR/exts/omni.isaac.ml_archive/pip_prebundle:$SCRIPT_DIR/extscache/omni.pip.torch-1_13_1-0.1.4+104.2.lx64/torch-1-13-1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SCRIPT_DIR/../../../$LD_LIBRARY_PATH:$SCRIPT_DIR/.:$SCRIPT_DIR/exts/omni.usd.schema.isaac/bin:$SCRIPT_DIR/kit:$SCRIPT_DIR/kit/kernel/plugins:$SCRIPT_DIR/kit/libs/iray:$SCRIPT_DIR/kit/plugins:$SCRIPT_DIR/kit/plugins/bindings-python:$SCRIPT_DIR/kit/plugins/carb_gfx:$SCRIPT_DIR/kit/plugins/rtx:$SCRIPT_DIR/kit/plugins/gpu.foundation:$SCRIPT_DIR/kit/extensions/extensions-bundled/bin # $SCRIPT_DIR/exts/omni.isaac.motion_planning/bin
