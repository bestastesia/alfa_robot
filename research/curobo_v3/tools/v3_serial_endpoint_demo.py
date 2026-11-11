from pathlib import Path

from v3_serial_endpoint_planner import SerialEndpointPlanner
import v3_visualized_task_demo as demo


SETTINGS=Path(__file__).resolve().parents[1]/'config/forward_precontact_wrist.json'
original_server=demo.viser.ViserServer


class TeleportPlanner(SerialEndpointPlanner):
    serial_endpoint_policy=True

    def __init__(self,args):
        super().__init__(args,reference_source=SETTINGS)
        self.extraction_teleport=True


def server_factory(*args,**kwargs):
    server=original_server(*args,**kwargs)
    server.gui.add_markdown('**独立串行终点实验，抽离仅瞬移。** Home→最近合法吸附构型→固定Updown/自由冗余求后撤35cm终点→距该终点最近合法运输构型→以运输构型为种子选龙头车终点；不因后续失败回头改选。连接用球腕参考＋起终关节加权LM先验＋首合法Informed Connect，不加联合通道优化。')
    server.gui.add_markdown('接触→抽离终点只显示一次瞬移，不生成抽离运动，不验中间碰撞；不能据此宣称完整抓取可执行。端点Pose/携箱碰撞和其他运动段仍硬验。当前仅预载3个代表验证：L22/R17、L03/R02、单臂右07；119全量尚未启动，因L03/R02出现新的3cm预接触解析无解，按用户要求停止排查策略变更。')
    return server


if __name__=='__main__':
    demo.FullCyclePlanner=TeleportPlanner
    demo.viser.ViserServer=server_factory
    demo.main(default_port=8201)
