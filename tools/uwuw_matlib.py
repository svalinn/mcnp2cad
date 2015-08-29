#!/usr/bin/env python
###############################################################################
# This script
###############################################################################
import argparse

from pyne.mcnp import mats_from_inp
from pyne.material import Material, MultiMaterial, MaterialLibrary

def uwuw_matlib(inp, out):
    mats = mats_from_inp(inp)
    print mats
    uwuw_mats = {}
    for mat in mats.values():
        if isinstance(mat, Material):
            mat_name = "mat:m{0}/rho:{1}".format(mat.metadata["mat_number"], mat.density)
            mat.metadata["name"] = mat_name
            uwuw_mats[mat_name] = mat
        if isinstance(mat, MultiMaterial):
           for m in mat._mats.keys():
               mat_name = "mat:m{0}/rho:{1}".format(m.metadata["mat_number"], m.density)
               m.metadata["name"] = mat_name
               uwuw_mats[mat_name] = m
           
    ml = MaterialLibrary(uwuw_mats)
    ml.write_hdf5(out)

def main():
    parser = argparse.ArgumentParser(description="This program reads")
    parser.add_argument("inp", action='store', help="Name of the MCNP input file.")
    parser.add_argument("-o", "--output", action='store', dest='out',
                       default="matlib.h5m", help="The name of the .h5m file to produce.")
    args = parser.parse_args()
    uwuw_matlib(args.inp, args.out)

if __name__ == '__main__':
    main()

