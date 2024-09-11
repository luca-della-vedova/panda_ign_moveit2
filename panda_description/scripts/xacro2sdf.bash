#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `panda_description` package

PKG_NAME="panda_description"
XACRO_PATH="$(ros2 pkg prefix --share $PKG_NAME)/urdf/panda.urdf.xacro"
SDF_PATH="$(ros2 pkg prefix --share $PKG_NAME)/panda/model.sdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=panda
    gripper:=true
    collision_arm:=true
    collision_gripper:=true
    ros2_control:=true
    ros2_control_plugin:=gz
    ros2_control_command_interface:=effort
    gazebo_preserve_fixed_joint:=false
)

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
ros2 run $PKG_NAME xacro2sdf_direct.bash "${XACRO_PATH}" "${XACRO_ARGS[@]}" "${@:1}" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"
