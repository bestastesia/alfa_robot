from ipc_left_prefix.smooth_timing import rt_curves


def preview_time(current, elapsed, duration, multiplier):
    if multiplier not in (1., 2., 5., 10.):
        raise ValueError('预览倍速必须为1、2、5或10')
    return min(duration, current + elapsed * multiplier)


def activate_preview(state, payload, timeline, play, kind):
    state['payload'] = payload
    state['curves'] = rt_curves(payload['points'])
    timeline.max = payload['report']['duration_s']
    timeline.value = 0.
    play.value = kind == 'quick_preview'


def quick_send_ready(state, allowed):
    payload = state.get('payload')
    return bool(allowed and payload and 'quick_ik_ms' in payload and state.get('curves')
        and state['status'].get('ready') and state['future'] is None)
