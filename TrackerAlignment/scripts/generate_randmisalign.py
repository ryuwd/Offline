import numpy as np
import pandas as pd
import csv

# generate an AlignedTracker Proditions file with purely random alignment parameters

def genshifts(nrows=36, max_shift=3.0):
    # uniform random shifts in range [-3.0, 3.0] mm
    shifts = np.random.uniform(low=-max_shift, high=max_shift, size=36*3).reshape((nrows, 3))
    rotations = np.zeros((nrows, 3))

    return np.concatenate((shifts, rotations), axis=1)

def main():
    planes = pd.DataFrame(genshifts())
    panels = pd.DataFrame(np.zeros((216, 6)))
    with open('Random_PlaneOnly.txt', 'w') as f:
        f.write('# uniform random shifts in range [-3.0, 3.0] mm\n\n')
        f.write("TABLE TrkAlignPlane\n")
        planes.to_csv(f, header=False, index=True)

        f.write("\n\nTABLE TrkAlignPanel\n")
        panels.to_csv(f, header=False, index=True)


if __name__ == "__main__":
    main()
