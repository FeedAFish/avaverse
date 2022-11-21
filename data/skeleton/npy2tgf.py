import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def npy_2_tgf(input, output):
    position = np.load(input)[..., 0]
    position_df = pd.DataFrame(position)
    position_df.index += 1
    content = position_df.to_string(header=False)
    content += "\n#\n"
    edge_df = pd.DataFrame(
        np.array(
            [
                [0, 1],
                [1, 2],
                [1, 5],
                [1, 8],
                [2, 3],
                [3, 4],
                [5, 6],
                [6, 7],
                [8, 9],
                [8, 12],
                [9, 10],
                [10, 11],
                [12, 13],
                [13, 14],
            ],
            dtype=int,
        )
        + 1
    )
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
