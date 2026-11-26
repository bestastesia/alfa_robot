"""CPU-only role/identity contract; proposed as curobo_core.handover_roles.

No joint reflection, cached solutions, planner construction, or GPU imports.
"""
from dataclasses import dataclass

SIDES = ('left', 'right')
PHYSICAL_IDS = {'left': 'wall_box_00', 'right': 'wall_box_01'}
NAMES = ['updown'] + [f'{s}_joint{i}' for s in SIDES for i in range(1, 8)]


def arm_slice(side):
    if side not in SIDES:
        raise ValueError('actual arm must be left or right')
    start = 1 + 7 * SIDES.index(side)
    return slice(start, start + 7)


@dataclass(frozen=True)
class HandoverRoles:
    giver: str = 'left'
    recipient: str = 'right'
    source: str = 'right'  # Original request identity, independent of actual arm.

    def __post_init__(self):
        if {self.giver, self.recipient} != set(SIDES) or self.source not in SIDES:
            raise ValueError('distinct actual arms and a known original physical source required')

    @property
    def pending(self):
        return next(s for s in SIDES if s != self.source)

    @property
    def owner_to_physical_id(self):
        return {self.giver: PHYSICAL_IDS[self.pending], self.recipient: PHYSICAL_IDS[self.source]}

    def parents(self, stage_index):
        if not isinstance(stage_index, int) or not 0 <= stage_index < 13:
            raise ValueError('thirteen-stage index required')
        result = {}
        if stage_index >= 2:
            result[PHYSICAL_IDS[self.source]] = (self.giver if stage_index <= 5 else self.recipient) + '_tool0'
        if stage_index >= 11:
            result[PHYSICAL_IDS[self.pending]] = self.giver + '_tool0'
        return result


def select_roles(picks):
    if set(picks) != set(SIDES):
        raise ValueError('two original physical requests required')
    source = max(SIDES, key=lambda s: picks[s].center_m[2])
    # ponytail: finite same-side proposal; try the other actual giver if this heuristic fails.
    giver = 'right' if picks[source].center_m[1] < 0 else 'left'
    return HandoverRoles(giver, next(s for s in SIDES if s != giver), source)


def admitted_precontacts(contacts, giver, check, limit=4):
    """Yield at most4 CHECKED full paths, retaining the existing finite raw pool.

    Try one representative per giver7 before its other recipient configurations.
    Keep all variants: a different held-arm posture can change physical collision.
    check(index,q) owns the unchanged task deadline and real shared stage gates.
    """
    from itertools import zip_longest
    import numpy as np
    if giver not in ('left','right') or not isinstance(limit,int) or isinstance(limit,bool) or limit<1:
        raise ValueError('actual giver and positive admitted-path budget required')
    rows=np.asarray(contacts)
    if rows.ndim!=2 or rows.shape[1]!=15 or not np.isfinite(rows).all():
        raise ValueError('finite standard15-axis contact pool required')
    offset=1 if giver=='left' else 8
    groups={}
    for i,q in enumerate(rows):
        key=tuple(np.round(q[offset:offset+7],7))
        groups.setdefault(key,[]).append(i)
    admitted=0
    for representatives in zip_longest(*groups.values()):
        for i in representatives:
            if i is None:continue
            path=check(i,rows[i])
            if path is None:continue
            yield i,rows[i],path
            admitted+=1
            if admitted==limit:return
