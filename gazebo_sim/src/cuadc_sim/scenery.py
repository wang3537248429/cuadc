"""仅包含可见场地设施，不向视觉模块传递目标答案。"""
from __future__ import annotations


def corner_flags_sdf(scenario) -> str:
    models = []
    for group in scenario.groups:
        color = "0.10 0.35 0.85" if group.name == "A" else "0.82 0.05 0.04"
        for index, (dn, de) in enumerate(((-30, -30), (-30, 30), (30, 30), (30, -30))):
            models.append(f'''
    <model name="zone_{group.name}_corner_{index}"><static>true</static>
      <pose>{group.center_north_m + dn:.5f} {-group.center_east_m - de:.5f} 0 0 0 0</pose>
      <link name="flag">
        <visual name="pole"><pose>0 0 0.90 0 0 0</pose>
          <geometry><cylinder><radius>0.018</radius><length>1.80</length></cylinder></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="cloth"><pose>0.27 0 1.58 0 0 0</pose>
          <geometry><box><size>0.52 0.012 0.34</size></box></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>
        </visual>
        <visual name="foot"><pose>0 0 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.14</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.35 0.36 0.34 1</ambient><diffuse>0.5 0.5 0.48 1</diffuse></material>
        </visual>
      </link>
    </model>''')
    return "\n".join(models)
