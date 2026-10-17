def tasks(task_rows=None):
    output = []
    task_rows = [4, 3, 2] if task_rows is None else task_rows
    left_columns = [4, 3, 2]
    right_columns = [0, 1, 2]
    for left_row in task_rows:
        for right_row in task_rows:
            if abs(left_row - right_row) > 1:
                continue
            for left_column in left_columns:
                for right_column in right_columns:
                    left_box = left_row * 5 + left_column
                    right_box = right_row * 5 + right_column
                    if left_box == right_box:
                        continue
                    label = (
                        f"L{left_box:02d}(排{5-left_row}/列{5-left_column}) · "
                        f"R{right_box:02d}(排{5-right_row}/列{5-right_column})"
                    )
                    output.append((label, {"left": left_box, "right": right_box}))
    return output
