#!/usr/bin/env python3
"""普通 GNSS 残差下的投水误差预算与独立蒙特卡洛检查。"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'onboard/src'))
from cuadc_onboard.precision import PrecisionBudget, estimate_impact_error
from cuadc_onboard.ballistics import solve_release


def main():
    import numpy as np
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'dist/validation/precision')
    parser.add_argument('--samples',type=int,default=5000)
    parser.add_argument('--height',type=float,default=16)
    parser.add_argument('--speed',type=float,default=12.8)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(261019);budget=PrecisionBudget()
    height,speed,delay,drag=args.height,args.speed,.12,.095
    estimate=estimate_impact_error(height,speed,delay,drag,budget)
    nominal=solve_release(0,0,height,speed,0,drag_rate_s=drag)
    errors=[]
    for _ in range(args.samples):
        # Shared absolute bias is applied to both fixes and cancels in the subtraction.
        common=rng.normal(0,5,2)
        relative=rng.normal(0,budget.gps_relative_sigma_m,2)
        camera=rng.normal(0,budget.camera_sigma_m,2)
        actual_speed=speed+rng.normal(0,budget.velocity_sigma_mps)
        actual_height=max(1,height+rng.normal(0,budget.altitude_sigma_m))
        actual_delay=max(0,delay+rng.normal(0,budget.delay_sigma_s))
        wind=rng.normal(0,budget.wind_sigma_mps,2)
        actual=solve_release(0,0,actual_height,actual_speed,0,*wind,drag_rate_s=max(0,drag+rng.normal(0,budget.drag_sigma_s)))
        fix_difference=(common+relative)-(common+camera)
        errors.append(fix_difference+np.array([actual.north_lead_m+actual_speed*actual_delay-nominal.north_lead_m-speed*delay,actual.east_lead_m]))
    errors=np.array(errors);radii=np.linalg.norm(errors,axis=1)
    result={'scope':'参数误差敏感性仿真；未使用真实飞行/实投样本，不代表实机命中率',
            'gps':'普通 GPS/北斗，残余相对位置标准差 3 m；5 m 公共固定偏差同时加在两次定位并抵消',
            'samples':args.samples,'seed':261019,'height_m':height,'speed_mps':speed,'assumptions':asdict(budget),'analytic':estimate,
            'empirical_r95_m':float(np.quantile(radii,.95)),
            'within_035m_fraction':float(np.mean(radii<=.35)),
            'analytic_bound_coverage':float(np.mean(radii<=estimate['r95_m'])),
            'target_tolerance_m':.35,'precision_requirement_met':bool(estimate['r95_m']<=.35)}
    (args.output/'ordinary-gnss.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(6.4,5.4),layout='constrained')
    ax.scatter(errors[:,0],errors[:,1],s=3,alpha=.15,color='#3178af',rasterized=True)
    for radius,color,label in [(.35,'#cf3d3d','Well tolerance: 0.35 m'),(estimate['r95_m'],'#176445','Predicted 95% bound')]:
        ax.add_patch(plt.Circle((0,0),radius,fill=False,color=color,lw=1.7,label=label))
    ax.set(xlabel='North impact error (m)',ylabel='East impact error (m)',title='Ordinary GNSS: relative error matters',aspect='equal')
    ax.grid(alpha=.15);ax.legend(fontsize=8)
    fig.savefig(args.output/'ordinary-gnss.png',dpi=160);plt.close(fig)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
