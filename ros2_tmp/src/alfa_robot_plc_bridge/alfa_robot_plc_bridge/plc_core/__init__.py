"""ROS-agnostic PLC communication core used by alfa_robot_plc_bridge."""

from .config import PlcConnectionConfig, PlcProtocolConfig
from .driver import PlcDriver
from .models import AxisCommandResult, AxisStatus, PlcSystemInfo
from .trajectory import (
    PositionOverwriteTrajectoryExecutor,
    TrajectoryCancelledError,
    TrajectoryExecutionConfig,
    TrajectoryExecutionReport,
    TrajectoryPoint,
)

__all__ = [
    "AxisCommandResult",
    "AxisStatus",
    "PlcConnectionConfig",
    "PlcDriver",
    "PlcProtocolConfig",
    "PlcSystemInfo",
    "PositionOverwriteTrajectoryExecutor",
    "TrajectoryCancelledError",
    "TrajectoryExecutionConfig",
    "TrajectoryExecutionReport",
    "TrajectoryPoint",
]
