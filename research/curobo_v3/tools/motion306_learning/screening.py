"""Selector-side exact screening: the environment's own deterministic gates, never search or retries.

Contact candidates run StageEnv.contact_checks lazily in ranked order (collision, extraction, single-arm
pre-approach); carry candidates run StageEnv.carry_checks once as a batch (payload collision, single-arm
envelope). Placement is gated only when the environment exposes gate reasons for it (LookaheadEnv place_gate).
The chosen action still executes once through StageEnv.step.
"""
import time

from motion311_learning.stage_env import EpisodeFailure


def screened_choice(env, order):
    """First index of `order` passing the current stage's gates (None when none does) and a screening record."""
    began = time.perf_counter()
    stage, rows = env.stage, env.observation.candidates
    rejected, checks, chosen, valid = {}, 0, None, None
    if stage == 0:
        # One batched collision pass, then the per-row deterministic gates lazily in rank order.
        free = env.contact_collision_free(rows) if hasattr(env, 'contact_collision_free') else None
        for index in order:
            checks += 1
            if free is not None and not free[int(index)]:
                rejected['contact_collision'] = rejected.get('contact_collision', 0) + 1
                continue
            try:
                env.contact_checks(rows[int(index)])
            except EpisodeFailure as failure:
                reason = failure.reason.split(':')[0]
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            chosen = int(index)
            break
    elif stage == 1:
        reasons = getattr(env, 'gate_reasons', None)  # LookaheadEnv already gated this pool when observing
        reasons = env.carry_checks(rows) if reasons is None else reasons
        checks, valid = len(reasons), sum(reason is None for reason in reasons)
        for reason in reasons:
            if reason is not None:
                rejected[reason] = rejected.get(reason, 0) + 1
        chosen = next((int(index) for index in order if reasons[int(index)] is None), None)
    else:
        reasons = getattr(env, 'gate_reasons', None)
        if reasons is None:
            chosen = int(order[0]) if len(order) else None
        else:
            checks, valid = len(reasons), sum(reason is None for reason in reasons)
            for reason in reasons:
                if reason is not None:
                    rejected[reason] = rejected.get(reason, 0) + 1
            chosen = next((int(index) for index in order if reasons[int(index)] is None), None)
    return chosen, dict(stage=stage, checks=checks, valid=valid, rejected=rejected,
                        screen_ms=(time.perf_counter() - began) * 1000)
