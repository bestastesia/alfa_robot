"""Configurable fixtures; box IDs are zero-based row-major from bottom/-Y."""
from dataclasses import dataclass
from decimal import Decimal
import math

from .scene import Pose, SceneObject

BOX_SIZE = (0.30, 0.40, 0.40)  # Must match the frozen payload sphere model.


@dataclass(frozen=True)
class WallLayout:
    rows: int = 5
    columns: int = 5
    distance_m: float = 0.9
    center_y_m: float = 0.0
    bottom_z_m: float = 0.0
    gap_m: float = 0.01
    active_box_ids: tuple | None = None

    def __post_init__(self):
        if (type(self.rows) is not int or type(self.columns) is not int or
                self.rows < 1 or self.columns < 1 or self.rows * self.columns > 1000):
            raise ValueError("wall must contain 1..1000 cells with integer rows/columns")
        values = (self.distance_m, self.center_y_m, self.bottom_z_m, self.gap_m)
        if not all(math.isfinite(v) for v in values) or self.distance_m <= 0 or self.gap_m < 0:
            raise ValueError("wall coordinates must be finite; distance > 0 and gap >= 0")
        ids = tuple(range(self.rows * self.columns)) if self.active_box_ids is None else tuple(self.active_box_ids)
        if (any(type(i) is not int or not 0 <= i < self.rows * self.columns for i in ids) or
                len(ids) != len(set(ids))):
            raise ValueError("active box IDs must be unique zero-based cells in this layout")
        object.__setattr__(self, "active_box_ids", tuple(sorted(ids)))

    def objects(self, front_x):
        if not math.isfinite(front_x):
            raise ValueError("chassis front X must be finite")
        # Preserve f044's literal 0.41 spacing exactly so default scene/cache IDs do not drift.
        pitch = float(Decimal(str(BOX_SIZE[1])) + Decimal(str(self.gap_m)))
        return tuple(SceneObject(f"wall_box_{i:02d}", BOX_SIZE, Pose((
            front_x + self.distance_m + BOX_SIZE[0] / 2,
            self.center_y_m + (i % self.columns - (self.columns - 1) / 2) * pitch,
            self.bottom_z_m + BOX_SIZE[2] / 2 + (i // self.columns) * pitch,
        ))) for i in self.active_box_ids)


def add_wall_arguments(parser):
    parser.add_argument("--wall-rows", type=int, default=5)
    parser.add_argument("--wall-columns", type=int, default=5)
    parser.add_argument("--wall-distance", type=float, default=0.9,
                        help="chassis front to near box face in metres, not map X")
    parser.add_argument("--wall-center-y", type=float, default=0.0)
    parser.add_argument("--wall-bottom-z", type=float, default=0.0)
    parser.add_argument("--wall-gap", type=float, default=0.01)
    parser.add_argument("--active-box-ids", type=int, nargs="*", default=None,
                        help="occupied zero-based cells; omitted means all, empty means none")


def wall_layout_from_args(args):
    return WallLayout(args.wall_rows, args.wall_columns, args.wall_distance,
                      args.wall_center_y, args.wall_bottom_z, args.wall_gap, args.active_box_ids)


def tasks(layout=None, task_rows=None):
    include_single = isinstance(layout, WallLayout)
    if layout is None:
        layout = WallLayout()
        task_rows = [4, 3, 2] if task_rows is None else task_rows
    elif not isinstance(layout, WallLayout):
        if task_rows is not None:
            raise TypeError("task rows were provided twice")
        task_rows, layout = layout, WallLayout()
    output = []
    occupied = set(layout.active_box_ids)
    available_rows = sorted({box_id // layout.columns for box_id in occupied}, reverse=True)
    task_rows = available_rows if task_rows is None else list(task_rows)
    if any(type(row) is not int or row not in available_rows for row in task_rows):
        raise ValueError("task rows must identify occupied rows in this wall layout")
    left_columns = range(layout.columns - 1, layout.columns // 2 - 1, -1)
    right_columns = range((layout.columns + 1) // 2)
    for left_row in task_rows:
        for right_row in task_rows:
            if abs(left_row - right_row) > 1:
                continue
            for left_column in left_columns:
                for right_column in right_columns:
                    left_box = left_row * layout.columns + left_column
                    right_box = right_row * layout.columns + right_column
                    if left_box == right_box or left_box not in occupied or right_box not in occupied:
                        continue
                    label = (
                        f"L{left_box:02d}(排{layout.rows-left_row}/列{layout.columns-left_column}) · "
                        f"R{right_box:02d}(排{layout.rows-right_row}/列{layout.columns-right_column})"
                    )
                    output.append((label, {"left": left_box, "right": right_box}))
    # Explicit layouts also expose sparse-wall and odd-column single-arm cleanup.
    if include_single:
        for pair in wall_sequence(layout):
            if len(pair) == 1:
                side, box_id = next(iter(pair.items()))
                row, column = divmod(box_id, layout.columns)
                output.append((f"{side[0].upper()}{box_id:02d}(排{layout.rows-row}/列{layout.columns-column}) · 单臂", pair))
    return output


def wall_sequence(layout):
    """Top to bottom, outer pairs first, then the centre/remaining singleton(s)."""
    occupied = set(layout.active_box_ids)
    output = []
    for rank, row in enumerate(sorted({i // layout.columns for i in occupied}, reverse=True)):
        left = [row * layout.columns + c for c in range(layout.columns-1, layout.columns//2, -1)
                if row * layout.columns + c in occupied]
        right = [row * layout.columns + c for c in range(layout.columns//2)
                 if row * layout.columns + c in occupied]
        if layout.columns % 2 == 0:
            middle = row * layout.columns + layout.columns//2
            if middle in occupied:
                left.append(middle)
        for index in range(max(len(left), len(right))):
            pair = {}
            if index < len(left):
                pair['left'] = left[index]
            if index < len(right):
                pair['right'] = right[index]
            output.append(pair)
        if layout.columns % 2:
            middle = row * layout.columns + layout.columns//2
            if middle in occupied:
                output.append({('left' if rank % 2 == 0 else 'right'): middle})
    return output
