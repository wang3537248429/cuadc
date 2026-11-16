#!/usr/bin/env python3
"""从公开模板生成有减速/降高过渡的仿真航线。"""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from cuadc_sim.mission import MissionItem, validate_mission
from cuadc_sim.recon_profile import ReconProfile, apply_recon_profile
from cuadc_sim.config import RECON_AIRSPEED_MPS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group',choices=('a','b'),default='a')
    parser.add_argument('--altitude',type=float,default=14)
    parser.add_argument('--airspeed',type=float,default=RECON_AIRSPEED_MPS)
    args=parser.parse_args()
    source=ROOT/f'config/missions/group_{args.group}.waypoints'
    items=[]
    for line in source.read_text().splitlines()[1:]:
        fields=line.split()
        if fields:
            items.append(MissionItem(seq=int(fields[0]),current=int(fields[1]),frame=int(fields[2]),command=int(fields[3]),
                                     **dict(zip(('param1','param2','param3','param4','x','y','z'),map(float,fields[4:11]))),
                                     autocontinue=int(fields[11])))
    items=apply_recon_profile(items,ReconProfile(altitude_m=args.altitude,airspeed_mps=args.airspeed))
    contract,errors=validate_mission(items)
    if errors:raise ValueError('; '.join(errors))
    lines=['QGC WPL 110']
    for item in items:
        values=[item.seq,item.current,item.frame,item.command,item.param1,item.param2,item.param3,item.param4,item.x,item.y,item.z,item.autocontinue]
        lines.append('\t'.join(str(value) for value in values))
    output=ROOT/f'config/missions/recon_low_{args.group}.waypoints'
    output.write_text('\n'.join(lines)+'\n')
    print(f'{output.name}: {args.altitude:g} m / {args.airspeed:g} m/s，任务协议通过')


if __name__=='__main__':main()
