#!/usr/bin/env python3
"""Compare only completed, same-area measured painting trials."""
import argparse
import json
from pathlib import Path
import numpy as np


def compare(root):
    reports=[json.loads((root/d/'report.json').read_text()) for d in ('horizontal','vertical')]
    metrics=[json.loads((root/d/'joint_metrics.json').read_text()) for d in ('horizontal','vertical')]
    if not all(m['passed'] for m in metrics):raise ValueError('Measured TCP paths must pass direction/length checks')
    if not all(r['passed'] and len(r['painting']['strokes'])==3 for r in reports):
        raise ValueError('Both trials must complete exactly three measured contact strokes')
    a,b=[np.array(r['accepted_work_area_corners']) for r in reports]
    if a.shape!=(4,3) or b.shape!=(4,3) or max(np.min(np.linalg.norm(a[:,None]-b[None,:],axis=2),axis=1))>.005:
        raise ValueError('Measured trial regions differ by more than 5mm')
    if metrics[0]['joint_names']!=metrics[1]['joint_names']:raise ValueError('Joint names differ')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names=metrics[0]['joint_names'];x=np.arange(len(names))
    fig,axes=plt.subplots(2,1,figsize=(11,8));summary={}
    for ax,group in zip(axes,('contact_stroke','noncontact_transfer')):
        summary[group]={}
        for i,(label,metric) in enumerate(zip(('Horizontal','Vertical'),metrics)):
            values=np.degrees(metric['groups'].get(group,{}).get('rotation_rad',[0]*len(names)))
            legend=(label+' contact' if group=='contact_stroke' else
                    ('Vertical transfer (horizontal paint)' if i==0 else 'Horizontal transfer (vertical paint)'))
            ax.bar(x+(i-.5)*.36,values,width=.36,label=legend)
            summary[group][label]=dict(rotation_deg=dict(zip(names,values.tolist())),ranking=[names[j] for j in np.argsort(-values)],duration_s=metric['groups'].get(group,{}).get('duration_s',0))
        ax.set_xticks(x,[n.removeprefix('right_manipulator_') for n in names]);ax.set_ylabel('Cumulative rotation (deg)');ax.set_title(group.replace('_',' '));ax.legend()
    fig.suptitle('Three-stroke comparison: same area, existing overlap and force')
    fig.tight_layout();fig.savefig(root/'comparison.png',dpi=160)
    (root/'comparison.json').write_text(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    compare(parser.parse_args().root)
