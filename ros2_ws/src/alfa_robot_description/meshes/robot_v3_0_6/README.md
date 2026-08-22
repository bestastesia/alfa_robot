# Robot V3.0.6 单臂模型

- 来源：`/mnt/mydisk/ALFA/backpack/robot_v3.0.6`。
- 上游只提供单臂；当前整机通过同一单臂宏实例化左右两套，安装到 V3.0.5 既有升降架轴线上。
- 外观与碰撞均使用上游八段 STL。
- 上游 URDF 的 ±90° 限位未采用；整机继续使用已确认的 J1/J3/J5/J7 ±180°、J2 ±105°、J4 ±145°、J6 ±120°。
- `robot.urdf`、`parts.json`、`user_model.json` 与 `export_report.json` 保留为来源追溯资产。
