"""Display-only failure playback. These frames are never controller trajectories."""
import rerun as rr


def replay_frames(payload):
    if not payload.get("success", False) and payload.get("diagnostic_frames"):
        return payload["diagnostic_frames"]
    return payload.get("frames", [])


def log_failure(diagnostic):
    if not diagnostic:
        return
    contacts = diagnostic.get("contacts", [])
    if contacts:
        rr.log("world/failure/contacts", rr.Points3D(
            [contact["position"] for contact in contacts],
            colors=[255, 30, 30], radii=0.025,
            labels=[" <-> ".join(contact["bodies"]) for contact in contacts]))
    if "target" in diagnostic:
        rr.log("world/failure/target", rr.Points3D(
            [diagnostic["target"]], colors=[255, 30, 30], radii=0.035,
            labels=["失败目标（不代表存在IK解）"]))
    rr.log("summary/failure", rr.TextLog(
        "诊断回放已停在失败点；非可执行轨迹。\n"
        + diagnostic.get("stage", "") + ": " + diagnostic.get("reason", "")
        + "\n" + diagnostic.get("snapshot", "")))
