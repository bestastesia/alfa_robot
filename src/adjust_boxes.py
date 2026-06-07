"""
调整 scene_v5.xml 箱子位置 + 集装箱自动适配
箱子尺寸 (半): 0.48m(X深) x 0.58m(Y宽) x 0.24m(Z高)
布局: 3行 x 4列 x 4层 = 48箱

用法: python adjust_boxes.py <row0_x> [间距]
示例: python adjust_boxes.py 1.2          # row0=1.2, 间距=0.62
      python adjust_boxes.py 1.2 0.7     # row0=1.2, 间距=0.7
"""
import sys
import re

BOX_HALF_X = 0.24
BOX_FULL_Y = 0.58
BOX_HALF_Z = 0.12
DOOR_X = 0.3
WALL_MARGIN = 0.3
ROWS, COLS, LAYERS = 3, 4, 4

COL_Y0 = -(COLS - 1) * BOX_FULL_Y / 2
LAYER_Z0 = BOX_HALF_Z + 0.05


def main():
    if len(sys.argv) < 2:
        print("用法: python adjust_boxes.py <row0_x> [间距]")
        print(f"  箱子: {BOX_HALF_X*2:.2f}x{BOX_FULL_Y:.2f}x{BOX_HALF_Z*2:.2f}m, {ROWS}行x{COLS}列x{LAYERS}层={ROWS*COLS*LAYERS}箱")
        sys.exit(1)

    x0 = float(sys.argv[1])
    gap = float(sys.argv[2]) if len(sys.argv) > 2 else BOX_HALF_X * 2 + 0.14

    row_x = [x0 + i * gap for i in range(ROWS)]
    col_y = [COL_Y0 + c * BOX_FULL_Y for c in range(COLS)]
    layer_z = [LAYER_Z0 + l * BOX_HALF_Z * 2 for l in range(LAYERS)]

    back_wall = row_x[-1] + BOX_HALF_X + WALL_MARGIN
    half_len = (back_wall - DOOR_X) / 2
    center_x = (DOOR_X + back_wall) / 2

    path = "scene_v5.xml"
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    # 箱子位置
    for r in range(ROWS):
        for c in range(COLS):
            for l in range(LAYERS):
                name = f'box_r{r}_c{c}_l{l}'
                pattern = rf'(<body name="{name}" pos=")[\d.]+ [\d.-]+ [\d.]+'
                repl = f'\\g<1>{row_x[r]:.2f} {col_y[c]:.2f} {layer_z[l]:.2f}'
                text = re.sub(pattern, repl, text)

    # 集装箱
    repl_map = {
        r'<geom name="c_floor".*?/>':
            f'<geom name="c_floor"      type="box" size="{half_len:.2f} 1.2 0.025"  pos="{center_x:.2f} 0 0.025"  rgba="0.55 0.45 0.35 1" contype="1" conaffinity="1"/>',
        r'<geom name="c_wall_left".*?/>':
            f'<geom name="c_wall_left"  type="box" size="{half_len:.2f} 0.05 1.3"   pos="{center_x:.2f} 1.25 1.3"  rgba="0.6 0.55 0.45 1" contype="1" conaffinity="1"/>',
        r'<geom name="c_wall_right".*?/>':
            f'<geom name="c_wall_right" type="box" size="{half_len:.2f} 0.05 1.3"   pos="{center_x:.2f} -1.25 1.3" rgba="0.6 0.55 0.45 1" contype="1" conaffinity="1"/>',
        r'<geom name="c_roof".*?/>':
            f'<geom name="c_roof"       type="box" size="{half_len:.2f} 1.2 0.04"   pos="{center_x:.2f} 0 2.64"    rgba="0.5 0.5 0.45 1" contype="1" conaffinity="1"/>',
        r'<geom name="c_wall_back".*?/>':
            f'<geom name="c_wall_back"  type="box" size="0.05 1.2 1.3"   pos="{back_wall:.2f} 0 1.3"     rgba="0.6 0.55 0.45 1" contype="1" conaffinity="1"/>',
        r'<geom name="c_outer_left".*?/>':
            f'<geom name="c_outer_left"  type="box" size="{half_len:.2f} 0.04 1.35" pos="{center_x:.2f} 1.32 1.35" rgba="0.25 0.55 0.25 1" contype="0" conaffinity="0" group="2"/>',
        r'<geom name="c_outer_right".*?/>':
            f'<geom name="c_outer_right" type="box" size="{half_len:.2f} 0.04 1.35" pos="{center_x:.2f} -1.32 1.35" rgba="0.25 0.55 0.25 1" contype="0" conaffinity="0" group="2"/>',
        r'<geom name="c_outer_roof".*?/>':
            f'<geom name="c_outer_roof"  type="box" size="{half_len:.2f} 1.25 0.03" pos="{center_x:.2f} 0 2.70"    rgba="0.25 0.55 0.25 1" contype="0" conaffinity="0" group="2"/>',
    }
    for pat, repl in repl_map.items():
        text = re.sub(pat, repl, text, flags=re.DOTALL)

    text = re.sub(r'<!-- Container:.*-->',
        f'<!-- Container: door at x={DOOR_X}, back at x={back_wall:.2f}, half-length={half_len:.2f}, center at x={center_x:.2f} -->', text)
    text = re.sub(r'<!-- Row 0:.*-->',
        f'<!-- Rows: {[f"{x:.2f}" for x in row_x]}  Cols Y: {[f"{y:.2f}" for y in col_y]}  Layers Z: {[f"{z:.2f}" for z in layer_z]} -->', text)

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)

    print(f"箱子: {BOX_HALF_X*2:.2f}x{BOX_FULL_Y:.2f}x{BOX_HALF_Z*2:.2f}m, {ROWS}x{COLS}x{LAYERS}={ROWS*COLS*LAYERS}箱")
    for r in range(ROWS):
        print(f"  Row {r}: x={row_x[r]:.2f}  front={row_x[r]-BOX_HALF_X:.2f}")
    print(f"  列 Y: {[f'{y:.2f}' for y in col_y]}")
    print(f"  层 Z: {[f'{z:.2f}' for z in layer_z]}")
    print(f"集装箱: {DOOR_X:.2f} -> {back_wall:.2f} (长{back_wall-DOOR_X:.2f}m)")


if __name__ == "__main__":
    main()
