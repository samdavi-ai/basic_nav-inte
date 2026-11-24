#!/usr/bin/env python3
"""Plot measured padded-box clearance versus time after the insertion request."""
import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_routing import separation


def series(folder):
    cfg=json.loads((folder/'scenario.json').read_text())
    box=cfg['box']
    if cfg['crossing_speed']:
        raise ValueError("Comparison requires a stationary inserted fixture")
    request=json.loads((folder/'world.jsonl').read_text().splitlines()[0])['request']
    with gzip.open(folder/'trace.jsonl.gz','rt') as stream:
        poses=np.unique([r['truth'] for r in map(json.loads,stream) if r['kind']=='motion' and r['truth']],axis=0)
    poses=poses[poses[:,0]>=request]
    return box, poses[:,0]-request,separation(poses[:,1:],('fixture',*box))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out',type=Path)
    parser.add_argument('folders',nargs='+',type=Path)
    args=parser.parse_args()
    fig,axes=plt.subplots(2,1,figsize=(8,5.7),layout='constrained')
    reference=None
    for folder in args.folders:
        box,t,clear=series(folder)
        if reference is not None and box!=reference:
            raise ValueError('Compared fixtures differ')
        reference=box
        for ax in axes:
            ax.plot(t,clear,label=folder.name,linewidth=1.6)
    for ax in axes:
        ax.axhline(0,color='black',linewidth=.8)
        ax.grid(alpha=.25)
        ax.set_ylabel('Clearance (m)')
    axes[0].set_title('Padded footprint to inserted box · Gazebo poses')
    axes[0].legend(ncols=len(args.folders))
    axes[1].set_xlim(0,7)
    axes[1].set_title('First seven seconds')
    axes[1].set_xlabel('Simulation seconds after insertion request')
    fig.savefig(args.out,dpi=160)
    plt.close(fig)


if __name__=='__main__':
    main()
