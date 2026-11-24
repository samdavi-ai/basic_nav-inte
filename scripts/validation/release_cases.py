"""Run the release-closure deterministic cases on the private Gazebo graph."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def cases():
    yield 'front', []
    for name, index in [('near_goal',9),('replan',4)]:
        yield name,['--index',str(index),'--synchronize-replan','--planner-fence']
    for name,index in [('cross',2),('mirrored_cross',7)]:
        yield name,['--index',str(index)]
    yield 'temporary',['--temporary']
    for name in ('blocked','original','larger','mirrored','study'):
        yield name,['--regression',name]
    for state in ('NAVIGATING','OBSTACLE_DETECTED','SAFETY_STOP','COSTMAP_UPDATE','ROUTE_REASSESSMENT','REPLANNING','PATH_VALIDATION','RESUMING'):
        yield 'stop_'+state,['--interrupt-at',state]
    yield 'moving_stop',['--regression','safety']


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('names',nargs='+',choices=[name for name,_ in cases()])
    args=parser.parse_args()
    for name,extra in cases():
        if args.names and name not in args.names:
            continue
        dest=args.root/name
        cmd=[sys.executable,'scripts/validation/dynamic_cases.py',str(dest),'--approach',*extra]
        with (args.root/(name+'.log')).open('x') as log:
            code=subprocess.run(cmd,stdout=log,stderr=log).returncode
            summary={'command':cmd,'exit':code}
            if (dest/'mission.json').exists() and not name.startswith('stop_'):
                dynamic=(dest/'world.jsonl').exists()
                if dynamic or (dest/'fixture.json').exists():
                    with (dest/('metrics.json' if dynamic else 'geometry.json')).open('x') as f:
                        summary['analysis_exit']=subprocess.run([sys.executable,'scripts/validation/'+('analyze_dynamic.py' if dynamic else 'analyze_routing.py'),str(dest)],stdout=f,stderr=log).returncode
            (args.root/(name+'_run.json')).write_text(json.dumps(summary,indent=2)+'\n')
