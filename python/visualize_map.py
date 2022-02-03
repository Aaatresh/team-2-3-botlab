import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser("")
parser.add_argument("--map_file", type=str)

args = parser.parse_args()

map_data = np.genfromtxt(args.map_file, delimiter=" ", skip_header = 1)

#global_origin = map_data[0, :]

plt.imshow(map_data[:, :])
plt.show()
