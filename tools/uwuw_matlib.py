#!/usr/bin/env python
###############################################################################
# This script
###############################################################################
import argparse

from pyne.mcnp import mats_from_inp
from pyne.material import Material, MultiMaterial, MaterialLibrary

def uwuw_matlib(inp, out):
    mats = mats_from_(inp)
    uwuw_mats = {}
    for mat in mats:
        if istype(mat, Material):
            mat_name = "mat:mat_"

    ml = MaterialLibrary(uwuw_mats)
    ml.write_hdf5("out")

def main():
    parser = argparse.ArgumentParser(description="This program reads")
    parser.add_argument("inp", action='store', help="Name of the MCNP input file.")
    parser.add_argument("-o", "--output", action='store', dest='out',
                       default="matlib.h5m", help="The name of the .h5m file to produce.")
    args = parser.parse_args()
    uwuw_matlib(args.inp, args.out)

if __name__ == '__main__':
    main()

