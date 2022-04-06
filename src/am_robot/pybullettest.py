import pybullet as p
import pybullet_data as pd
import os
import argparse

parser = argparse.ArgumentParser(formatter_class=argparse.MetavarTypeHelpFormatter,
        description=('''Pybullet v-Hacd segmentation of .obj file'''),
        add_help=True)

parser.add_argument('--i', default='padlock.obj', type=str, help='input .obj file')
parser.add_argument('--o', default='padlock_vhacd.obj', type=str, help='output .obj file')
parser.add_argument('--log', default='padlock_log.txt', type=str, help='output log.txt file')
parser.add_argument('--res', default=100000, type=int, help='Output resolution, maximum number of voxels')

args = parser.parse_args()

p.connect(p.DIRECT)
name_in = os.path.join(pd.getDataPath(), args.i)

cwd = os.getcwd()

name_out = os.path.join(cwd,'output',args.o)
name_log = os.path.join(cwd,'output',args.log)

p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=args.res)
