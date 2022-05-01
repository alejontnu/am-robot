import pybullet as p
import pybullet_data as pd
import os
import argparse

parser = argparse.ArgumentParser(formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Pybullet v-Hacd segmentation of .obj file'''),
        add_help=True)

parser.add_argument('--i', default='slopesquarecylinder.obj', type=str, help='input .obj file')
parser.add_argument('--o', default='slopesquarecylinder_vhacd.obj', type=str, help='output .obj file')
parser.add_argument('--log', default='slopesquarecylinder_log.txt', type=str, help='output log.txt file')
parser.add_argument('--res', default=16000000, type=int, help='Output resolution, maximum number of voxels')

args = parser.parse_args()

p.connect(p.DIRECT)

cwd = os.getcwd()

#name_in = os.path.join(pd.getDataPath(), args.i)
name_in = os.path.join(cwd,'output',args.i)

name_out = os.path.join(cwd,'output',args.o)
name_log = os.path.join(cwd,'output',args.log)

p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=args.res)
