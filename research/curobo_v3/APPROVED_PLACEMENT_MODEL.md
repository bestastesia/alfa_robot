# 升降架20球与侧吸搬运目标

本合同固定2026-10-13用户确认的研究模型与目标，不代表实机精确碰撞或全部94任务验收。

- `config/arm_carriage_collision_frozen.json`：MorphIt20球、局部arm_carriage坐标、米制，iterations=1000、coverage_weight=1000、protrusion_weight=100。只替换固定15轴与mobile18研究配置的升降架球；装配、其他球、限位不变。
- side：箱心高度1.20m、world Y倾斜45°，固定附着变换反算TCP为(0.443933983,±0.425,1.093933983)m，wxyz=(0,0.382683432,0,0.923879533)。这是导航重定位前的搬运构型目标，最终龙头车释放目标另行配置。
- side atlas：`generated/selected_placement_20261013/all_solutions.npz`，2816解，不仅加载代表集。放置Updown网格-0.20到0每2cm，ψ每10°，存档包络≤1.05m；全部行需FK/合同验证，运行时按当前任务重新检查碰撞。
- 吸附候选仍为Updown每5cm、ψ每10°，按原Home加权距离保留64。Updown10cm等价关节15°。不要把放置网格套到接触。
- top仍为TCP(0.55,±0.425,1.0)m、wxyz=(0,1,0,0)，沿用原atlas。

模型来源为运控工程师1.3确认的冻结JSON；atlas来自`handoff-placement-9DVWP4.md`指定的解析全集。生成报告是历史固定L22/R17场景数据，不代表任意场景全集；球拟合也不是精确STL安全证明。

所有内容哈希登记在`config/research_asset_manifest.json`，缓存身份包含模型、目标、atlas、场景和负载。更换路径后运行`tools/prepare_checkout.py`重定位并编译解析桥；不要覆盖其他工程师URDF装配改动。
