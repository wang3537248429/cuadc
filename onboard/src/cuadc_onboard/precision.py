"""动态投水的不确定度预算；不把 GPS 标称精度当作命中精度。"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from .ballistics import solve_release


@dataclass(frozen=True)
class PrecisionBudget:
    gps_relative_sigma_m: float = 3.0
    camera_sigma_m: float = .3
    altitude_sigma_m: float = .5
    velocity_sigma_mps: float = .3
    wind_sigma_mps: float = .5
    delay_sigma_s: float = .03
    drag_sigma_s: float = .02

    @classmethod
    def from_release(cls, release):
        return cls(gps_relative_sigma_m=release.gps_relative_sigma_m,camera_sigma_m=release.camera_position_sigma_m,
                   altitude_sigma_m=release.altitude_sigma_m,velocity_sigma_mps=release.velocity_sigma_mps,
                   wind_sigma_mps=release.wind_sigma_mps,delay_sigma_s=release.actuator_delay_sigma_s,drag_sigma_s=release.drag_sigma_s)


def estimate_impact_error(altitude_m, speed_mps, delay_s, drag_rate_s, budget: PrecisionBudget):
    """Numerical Jacobian of the same ballistic solver used for the command.

    GPS term means the *difference* between target-fix and release-fix errors.
    Common constant bias cancels; temporal drift does not. The conservative
    default must be replaced by measured residuals before precision release.
    """
    values=asdict(budget)
    if any(not math.isfinite(x) or x<0 for x in values.values()):
        raise ValueError('精度预算必须是非负有限数值')
    if altitude_m<=0 or not math.isfinite(speed_mps):raise ValueError('飞行状态无效')
    def lead(height=altitude_m,speed=speed_mps,wind=0,drag=drag_rate_s):
        result=solve_release(0,0,height,speed,0,wind_north_mps=wind,drag_rate_s=drag)
        return result.north_lead_m+speed*delay_s
    h_step=min(.05,altitude_m/4)
    height_gain=(lead(height=altitude_m+h_step)-lead(height=altitude_m-h_step))/(2*h_step)
    speed_gain=(lead(speed=speed_mps+.01)-lead(speed=speed_mps-.01))/.02
    wind_gain=(lead(wind=.01)-lead(wind=-.01))/.02
    drag_step=min(.001,max(1e-5,drag_rate_s/2))
    drag_gain=(lead(drag=drag_rate_s+drag_step)-lead(drag=max(0,drag_rate_s-drag_step)))/(drag_step+min(drag_step,drag_rate_s))
    contributions={
        'gps_relative':budget.gps_relative_sigma_m,
        'camera':budget.camera_sigma_m,
        'altitude':abs(height_gain)*budget.altitude_sigma_m,
        'velocity':abs(speed_gain)*budget.velocity_sigma_mps,
        'wind':abs(wind_gain)*budget.wind_sigma_mps,
        'delay':abs(speed_mps)*budget.delay_sigma_s,
        'drag':abs(drag_gain)*budget.drag_sigma_s,
    }
    along_variance=sum(v*v for v in contributions.values())
    cross_variance=budget.gps_relative_sigma_m**2+budget.camera_sigma_m**2+(wind_gain*budget.wind_sigma_mps)**2
    return {'r95_m':math.sqrt(-2*math.log(.05))*math.sqrt(max(along_variance,cross_variance)),
            'along_sigma_m':math.sqrt(along_variance),'cross_sigma_m':math.sqrt(cross_variance),
            'components_sigma_m':contributions,'scope':'一阶独立高斯误差预算，需实投验证；不含未知系统偏差'}
