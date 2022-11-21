import argparse
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

RESOLUTION = (512, 512)

BASE_JOINT_NAMES = [
    "Head",
    "Neck",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "Hips",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
]
MIXAMO_JOINT_NAMES = ["mixamorig:" + x for x in BASE_JOINT_NAMES]


def fbx_2_npy_frame(i, keypoints):
    bone_struct = bpy.data.objects["Armature"].pose.bones
    armature = bpy.data.objects["Armature"]

    for j, name in enumerate(MIXAMO_JOINT_NAMES):
        global_location = (
            armature.matrix_world @ bone_struct[name].matrix @ Vector((0, 0, 0))
        )
        keypoints[j, :, i] = global_location


def fbx_2_npy(input, output):
    if bpy.data.objects.get("Cube") is not None:
        cube = bpy.data.objects["Cube"]
        bpy.data.objects.remove(cube)

    if bpy.data.objects.get("Light") is not None:
        bpy.data.objects["Light"].data.energy = 2
        bpy.data.objects["Light"].data.type = "POINT"

    bpy.data.scenes["Scene"].render.resolution_x = RESOLUTION[0]
    bpy.data.scenes["Scene"].render.resolution_y = RESOLUTION[1]
    bpy.data.scenes["Scene"].render.resolution_percentage = 100

    bpy.ops.import_scene.fbx(filepath=str(input))

    keypoints = np.empty(
        (
            len(MIXAMO_JOINT_NAMES),
            3,
            (
                int(bpy.data.actions[0].frame_range[1])
                if len(bpy.data.actions) == 1
                else 0
            )
            + 1,
        )
    )

    if len(bpy.data.actions):
        for i in range(keypoints.shape[2]):
            bpy.context.scene.frame_set(i)
            fbx_2_npy_frame(i, keypoints)
    else:
        fbx_2_npy_frame(0, keypoints)

    np.save(output, keypoints)


def main():
    parser = argparse.ArgumentParser(prog="fxb2npy")
    parser.add_argument("-i", "--input", default="fbx", type=Path)
    parser.add_argument("-o", "--output", default="npy", type=Path)
    args = parser.parse_args()
    fbx_2_npy(args.input, args.output)


if __name__ == "__main__":
    main()
