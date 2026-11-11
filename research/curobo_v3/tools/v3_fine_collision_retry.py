import time


class FineCollisionRetry(RuntimeError):
    def __init__(self, frames, first_bad_index):
        super().__init__('candidate path failed original-model fine collision validation')
        self.frames = frames
        self.first_bad_index = first_bad_index


def search_with_fine_retry(search, start, goals, lower, upper, validity, budget, seed, **kwargs):
    original_mask = validity.mask
    had_instance_mask = 'mask' in validity.__dict__
    original_resolution = getattr(validity, 'search_edge_resolution', None)
    started = time.perf_counter()
    attempts = []

    def interrupt_on_fine_collision(values):
        mask = original_mask(values)
        if not bool(mask.all().item()):
            raise FineCollisionRetry(len(values), int((~mask).nonzero()[0].item()))
        return mask

    try:
        validity.search_edge_resolution = .05
        validity.mask = interrupt_on_fine_collision
        try:
            path, stats = search(start, goals, lower, upper, validity, budget, seed, **kwargs)
            attempts.append(dict(resolution=.05, ms=(time.perf_counter()-started)*1000, trigger=None))
        except FineCollisionRetry as collision:
            elapsed = time.perf_counter()-started
            attempts.append(dict(resolution=.05, ms=elapsed*1000, trigger='fine_collision',
                                 fine_path_frames=collision.frames, first_bad_frame=collision.first_bad_index+1))
            remaining = max(0., budget-elapsed)
            validity.mask = original_mask
            if remaining > 0:
                validity.search_edge_resolution = .03
                retry_started = time.perf_counter()
                path, stats = search(start, goals, lower, upper, validity, remaining, seed, **kwargs)
                attempts.append(dict(resolution=.03, ms=(time.perf_counter()-retry_started)*1000,
                                     budget_ms=remaining*1000, trigger='retry_after_fine_collision'))
            else:
                path, stats = None, dict(search_ms=elapsed*1000, failure_reason='fine_collision_at_budget_end')
        stats = dict(stats)
        stats['last_attempt_search_ms'] = stats.get('search_ms')
        stats['search_ms'] = (time.perf_counter()-started)*1000
        stats['fine_collision_retry'] = dict(triggered=attempts[0]['trigger']=='fine_collision', attempts=attempts,
                                             total_ms=(time.perf_counter()-started)*1000, total_budget_ms=budget*1000,
                                             final_validation='original spheres, original fine density; no collision tolerance')
        return path, stats
    finally:
        if had_instance_mask:
            validity.mask = original_mask
        else:
            del validity.mask
        if original_resolution is None:
            del validity.search_edge_resolution
        else:
            validity.search_edge_resolution = original_resolution
