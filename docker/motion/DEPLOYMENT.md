# Motion Domain Release Deployment

## Prerequisites

- x86_64 Ubuntu host with Docker Engine and Compose plugin.
- Host CPU set `21,22` available; CPU 14 is reserved for RT-Control.
- RT-Control and other domains use `ROS_DOMAIN_ID=7` and the central interface contract at `92d6ff2ed0b45684d7da2170d96703ca8be569f4`.
- RT-Control exposes `/joint_states`, `/tf`, `/tf_static`, and `/whole_body_jtc/follow_joint_trajectory`.

## Install

```bash
# 单文件归档：
zstd -dc motion-domain-2.0.3.tar.zst | docker load

# 若 Release 因 GitHub 单文件大小限制分片：
cat motion-domain-2.0.3.tar.zst.part-* | zstd -dc | docker load

export MOTION_IMAGE=alfa-motion:2.0.3
docker compose up -d motion
docker compose ps
docker compose logs -f motion
```

The image is immutable and does not compile or clone code during startup. Runtime data and logs use Docker named volumes.

## Motion-specific runtime storage

| Volume | Purpose | Host device/capability impact |
| --- | --- | --- |
| `motion-data` | Planner sessions, generated trajectories, and task diagnostics required for post-run analysis | None |
| `motion-logs` | ROS 2 and Motion process logs required for health diagnosis | None |

Motion requires no hardware device mapping, Linux capability, realtime limit, or published TCP/UDP port.

## Verify

```bash
docker run --rm "$MOTION_IMAGE" robot-runtime-doctor
docker run --rm "$MOTION_IMAGE" contract-runtime-doctor
docker run --rm "$MOTION_IMAGE" release-info
docker inspect --format '{{.State.Health.Status}}' "$(docker compose ps -q motion)"
```

The Motion container is healthy while idle or busy. Missing/stale `/joint_states` or a dead planner marks it unhealthy.

## Stop and rollback

```bash
docker compose down
export MOTION_IMAGE=<previous immutable image tag or digest>
docker compose up -d motion
```

Motion owns no hardware device and does not start, reset, enable, or stop RT-Control.
