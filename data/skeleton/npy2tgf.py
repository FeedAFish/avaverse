import argparse
from pathlib import Path

import numpy as np
import pandas as pd

SKELETON_BONES = np.array(
    [
        [0, 4],
        [4, 5],
        [5, 6],
        [0, 1],
        [1, 2],
        [2, 3],
        [0, 7],
        [7, 8],
        [8, 9],
        [9, 10],
        [8, 11],
        [11, 12],
        [12, 13],
        [8, 14],
        [14, 15],
        [15, 16],
    ],
    dtype=int,
)


def npy_2_tgf(input, output):
    position = np.load(input)[..., 0]
    position_df = pd.DataFrame(position)
    position_df.index += 1
    content = position_df.to_string(header=False)
    content += "\n#\n"
    edge_df = pd.DataFrame(SKELETON_BONES + 1)
    content += edge_df.to_string(index=False, header=False)
    content += "\n#\n"
    with open(output, "w") as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser(prog="npy2tgf")
    parser.add_argument("-i", "--input", default="fbx", type=Path)
    parser.add_argument("-o", "--output", default="npy", type=Path)
    args = parser.parse_args()
    npy_2_tgf(args.input, args.output)


if __name__ == "__main__":
    main()
