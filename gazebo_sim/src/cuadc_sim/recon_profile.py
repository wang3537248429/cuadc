"""低空侦察任务配置：进入航段前减速降高，退出后恢复巡航。"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pymavlink import mavutil

from .config import SCAN_END_TAG, SCAN_START_TAG, RECON_AIRSPEED_MPS, RECON_TURN_AIRSPEED_MPS
from .mission import (JUMP_TAG, MissionItem, NAV_TAKEOFF, NAV_WAYPOINT,
                      latlon_to_ne, ne_to_latlon, validate_mission)


@dataclass(frozen=True)
class ReconProfile:
    altitude_m: float = 14.0
    airspeed_mps: float = RECON_AIRSPEED_MPS
    cruise_airspeed_mps: float = RECON_TURN_AIRSPEED_MPS
    minimum_airspeed_mps: float = 9.0
    bank_limit_deg: float = 35.0
    scan_bank_deg: float = 10.0

    def validate(self) -> None:
        values=(self.altitude_m,self.airspeed_mps,self.cruise_airspeed_mps,self.minimum_airspeed_mps,self.bank_limit_deg,self.scan_bank_deg)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('侦察参数必须是有限数值')
        if (not 10 <= self.altitude_m <= 20 or self.minimum_airspeed_mps<=0
                or not 0 < self.scan_bank_deg <= self.bank_limit_deg <= 45):
            raise ValueError('仿真侦察高度为 10..20 m，坡度限值为 0..45°')
        # AIRSPEED_MIN is the configured conservative flight envelope, not a measured stall speed.
        straight_floor=self.minimum_airspeed_mps/math.sqrt(math.cos(math.radians(self.scan_bank_deg)))
        turning_floor=self.minimum_airspeed_mps/math.sqrt(math.cos(math.radians(self.bank_limit_deg)))
        if self.airspeed_mps < straight_floor*1.02 or self.cruise_airspeed_mps<turning_floor*1.025:
            raise ValueError('直线/转弯空速不足以保留对应坡度下的裕量')
        if self.cruise_airspeed_mps < self.airspeed_mps:
            raise ValueError('巡航空速不能低于侦察空速')


def apply_recon_profile(items: list[MissionItem], profile: ReconProfile) -> list[MissionItem]:
    """Create a new mission; never silently modify an uploaded mission in flight."""
    profile.validate()
    start=next(i for i in items if i.command==JUMP_TAG and round(i.param1)==SCAN_START_TAG)
    end=next(i for i in items if i.command==JUMP_TAG and round(i.param1)==SCAN_END_TAG)
    lanes=[i for i in items if start.seq<i.seq<end.seq and i.command==NAV_WAYPOINT]
    if not lanes:
        raise ValueError('缺少侦察航点')
    first=lanes[0]
    # DO_CHANGE_SPEED param1=0 is AIRSPEED, param3=-1 preserves throttle management.
    speed=MissionItem(seq=0,current=0,autocontinue=1,frame=2,command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                      param1=0,param2=profile.airspeed_mps,param3=-1,param4=0)
    origin=items[0]
    first_n,first_e=latlon_to_ne(first.x,first.y,origin.x,origin.y)
    last_n,last_e=latlon_to_ne(lanes[-1].x,lanes[-1].y,origin.x,origin.y)
    length=math.hypot(last_n-first_n,last_e-first_e)
    if length<1:raise ValueError('侦察入口与出口必须分开至少一米')
    unit_n,unit_e=(last_n-first_n)/length,(last_e-first_e)/length
    # Finish 5 m earlier to reserve tracking room before the bounded reversal;
    # the down camera still covers the final wells before this end point.
    last_n-=unit_n*5
    last_e-=unit_e*5
    radius=max(70.0,profile.cruise_airspeed_mps**2/(9.80665*math.tan(math.radians(profile.bank_limit_deg)))*1.10)
    settle_n=first_n-unit_n*60+unit_e*radius
    settle_e=first_e-unit_e*60-unit_n*radius
    settle_lat,settle_lon=ne_to_latlon(settle_n,settle_e,origin.x,origin.y)
    settle=replace(first,command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
                   param1=0,param2=-radius,param3=0,param4=1,
                   x=settle_lat,y=settle_lon,z=profile.altitude_m)
    output=[]
    for item in items:
        if item.command==NAV_TAKEOFF:
            item=replace(item,param1=10,z=profile.altitude_m)
        if item.seq==start.seq:
            one_turn=replace(settle,command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                             param1=1,param2=0,param3=-radius,param4=1)
            turn_speed=replace(speed,param2=profile.cruise_airspeed_mps)
            output.extend([turn_speed,one_turn,turn_speed,settle,speed,replace(first,z=profile.altitude_m)])
        if start.seq<item.seq<end.seq and item.command==NAV_WAYPOINT:
            item=replace(item,z=profile.altitude_m)
            if item.seq==lanes[-1].seq:
                lat,lon=ne_to_latlon(last_n,last_e,origin.x,origin.y)
                item=replace(item,x=lat,y=lon)
        if item.seq==end.seq+1:
            n,e=latlon_to_ne(item.x,item.y,origin.x,origin.y)
            lat,lon=ne_to_latlon(n-unit_n*5,e-unit_e*5,origin.x,origin.y)
            item=replace(item,x=lat,y=lon)
        if item.seq==end.seq+2:
            # Retain the four-slot protocol while widening its fallback hold.
            hold_offset=radius+20
            hold_lat,hold_lon=ne_to_latlon(last_n-unit_n*20+unit_e*hold_offset,
                                          last_e-unit_e*20-unit_n*hold_offset,origin.x,origin.y)
            item=replace(item,param3=-radius,x=hold_lat,y=hold_lon)
        # Keep decision and four patch slots contiguous for the online release contract.
        if item.seq==end.seq+6:
            output.append(replace(speed,param2=profile.cruise_airspeed_mps))
        output.append(item)
    output=[replace(item,seq=index) for index,item in enumerate(output)]
    contract,errors=validate_mission(output)
    if errors:raise ValueError('; '.join(errors))
    # Two tangent arcs and a straight approach avoid a tight low-speed S turn.
    # Arbitrary uploaded routes are never rewritten by this utility at runtime.
    anchor=output[contract.fallback_exit_seq]
    anchor_n,anchor_e=latlon_to_ne(anchor.x,anchor.y,origin.x,origin.y)
    if abs(unit_e)>.1 or abs(anchor_n)<140:
        raise ValueError('此示例返航规划只支持公开南/北直线模板')
    sign=1 if anchor_n>0 else -1
    if abs(anchor_n)<2*radius:raise ValueError('返航空间不足')
    tail=[replace(speed,param2=profile.cruise_airspeed_mps)]
    lat,lon=ne_to_latlon(anchor_n,anchor_e+radius,origin.x,origin.y)
    tail.append(replace(first,command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                        param1=.5,param2=0,param3=sign*radius,param4=1,x=lat,y=lon,z=14))
    lat,lon=ne_to_latlon(sign*radius,anchor_e+2*radius,origin.x,origin.y)
    tail.append(replace(first,x=lat,y=lon,z=9,param2=3))
    lat,lon=ne_to_latlon(sign*radius,anchor_e+radius,origin.x,origin.y)
    tail.append(replace(first,command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                        param1=.25,param2=0,param3=sign*radius,param4=1,x=lat,y=lon,z=5))
    lat,lon=ne_to_latlon(0,55,origin.x,origin.y)
    tail.append(replace(first,x=lat,y=lon,z=3,param2=3))
    lat,lon=ne_to_latlon(0,20,origin.x,origin.y)
    tail.append(replace(items[-1],x=lat,y=lon,z=0))
    output=output[:contract.fallback_exit_seq+1]+tail
    return [replace(item,seq=index) for index,item in enumerate(output)]
