from v3_polar_refined_cycle import PolarRefinedCyclePlanner, guidance_summary
from v3_current_urdf_forward import current_assets
from v3_contact_roll_policy import aligned_grasp_transform
import v3_guided_cycle_planner
import v3_visualized_task_demo as demo


if __name__ == '__main__':
    v3_guided_cycle_planner.guidance_summary = guidance_summary
    demo.grasp_transform = aligned_grasp_transform
    demo.PLAYBACK_SCOPES.clear()
    demo.PLAYBACK_SCOPES.update({'全流程': None, '搜索1 · Home→预接触': 'approach',
        '搜索2 · 前向接近': 'contact_approach', '搜索3 · 抽离': 'extract',
        '搜索4 · 运输收束': 'transport', '搜索5 · 龙头车放箱': 'conveyor_place',
        '搜索6 · 释放后回Home': 'return_home'})
    demo.PHASE_NAMES['contact_approach'] = '解析＋整段30轮引导前向接近'
    demo.PHASE_NAMES['extract'] = '解析＋整段30轮引导抽离（途中空载/终点携箱验碰）'
    demo.main(default_port=8221, default_assets=current_assets(), planner_class=PolarRefinedCyclePlanner)
