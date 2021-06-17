#!/usr/bin/env bash

# Expose the X server on the host.

xhost +local:root

# --rm: Make the container ephemeral (delete on exit).
# -it: Interactive TTY.
# --gpus all: Expose all GPUs to the container.

docker run \
  --rm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  fbottarel/manip-env-visu \
  /workspace/manip-env-visu/build/examples/gripper_render /workspace/manip-env-visu/models/hands/franka_gripper/franka_gripper.urdf /workspace/manip-env-visu/models/objects/cracker_box/textured_simple.obj /workspace/manip-env-visu/models/objects/cracker_box/texture_map.png

# Fix xhost permissions
xhost -local:root
