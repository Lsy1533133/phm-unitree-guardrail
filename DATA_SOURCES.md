# DATA SOURCES (all public)

- Joint limits: parsed from OFFICIAL Unitree URDFs in unitreerobotics/unitree_ros
  (fetched 2026-10-13):
  - Z1 arm:  robots/z1_description/xacro/z1.urdf
  - Go2:     robots/go2_description/urdf/go2_description.urdf
  - G1 29d:  robots/g1_description/g1_29dof.urdf
- LowCmd field semantics: unitreerobotics/unitree_sdk2_python (q/dq/tau per joint).
- Demo stream: SYNTHETIC, generated within the official limits above (spec-accurate
  simulator caliber, like our M30T work). No real robot field data. No external egress.
