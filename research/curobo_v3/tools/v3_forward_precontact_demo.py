from v3_current_urdf_forward import CurrentUrdfForwardPlanner, current_assets
from v3_contact_roll_policy import aligned_grasp_transform
import v3_visualized_task_demo as demo


original_server = demo.viser.ViserServer


def server_factory(*args, **kwargs):
    server = original_server(*args, **kwargs)
    server.gui.add_markdown('**直接预接触IK＋前向接近：117组任务按需实时计算。** 本页不预载整组规划结果。选择任务后点击“计算完整流程并播放”，只计算当前任务；每次重新生成候选和路径，求解器/检查器资源复用。')
    server.gui.add_markdown('任务吸附6D Pose→沿吸盘法向外退3cm得到预接触Pose→参考初始构型求预接触IK→Home规划到位→参考实际到达构型求吸附IK→LM先验＋Informed Connect前向接近。没有从吸附关节构型反推3cm运动，也不会先验证下一段再回选上一段。')
    server.gui.add_markdown('之后继续实际35cm抽离、运输收束、导航到位、龙头车放箱、释放和回Home。可看完整流程或六段独立轨迹/TCP朝向；偏线和过程朝向只作软代价及观察，不作硬拒绝。')
    server.gui.add_markdown('保持原碰撞规则：Home到预接触时目标箱参与碰撞，最后接近段排除目标箱；35cm抽离途中空载验碰、终点携箱，导航仍到位瞬移。不代表全程携箱或实机验收。已做3个代表任务联通验证，未批测117组。')
    return server


if __name__ == '__main__':
    demo.PLAYBACK_SCOPES.clear()
    demo.PLAYBACK_SCOPES.update({'全流程': None, '搜索1 · Home→直接预接触IK': 'approach', '搜索2 · 前向3cm接近': 'contact_approach',
        '搜索3 · 35cm抽离': 'extract', '搜索4 · 抽离→运输位': 'transport',
        '搜索5 · 龙头车放箱': 'conveyor_place', '搜索6 · 释放后回Home': 'return_home'})
    demo.PHASE_NAMES['contact_approach'] = '前向LM软参考接近吸附（目标箱排除）'
    demo.PHASE_NAMES['extract'] = 'LM软参考后撤35cm（途中空载验碰，终点携箱）'
    demo.FullCyclePlanner = CurrentUrdfForwardPlanner
    demo.grasp_transform = aligned_grasp_transform
    demo.viser.ViserServer = server_factory
    demo.main(default_port=8210, default_assets=current_assets())
