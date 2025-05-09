import numpy as np

def normalize_arc(start, end):
    start %= 2 * np.pi
    end %= 2 * np.pi
    if end < start:
        end += 2 * np.pi
    return start, end

def arc_diff(a1, a2):
    return (a2 - a1) % (2 * np.pi)

def splitter(arc):
    split=[]
    if arc[1] > 2 * np.pi:
        split.append((arc[0], 2 * np.pi))
        split.append((0, arc[1] - 2 * np.pi))
    else:
        split.append((arc[0], arc[1]))
    return split

def get_unoccluded_segments(new_arc, existing_arcs):
    unoccluded = []
    
    new_arcs = splitter(new_arc)
    for new_arc in new_arcs:
        s1, e1 = normalize_arc(*new_arc)

        covered = []
        for s2, e2 in existing_arcs:
            s2, e2 = normalize_arc(s2, e2)
            covered.append((s2, e2))

        covered.sort()
        current = s1
        for s2, e2 in covered:
            if e2 <= current:
                continue
            if s2 > e1:
                break
            if s2 > current:
                unoccluded.append((current, min(s2, e1)))
            current = max(current, e2)

        if current < e1:
            unoccluded.append((current, e1))
    return unoccluded

def get_angle(x, y, r, angle, building):
    bx, by = building
    dx, dy = x - bx, y - by
    b = 2*((dy)*np.sin(angle)+(dx)*np.cos(angle))
    d = np.hypot(dx, dy)
    c = d*d - r**2
    a = 1
    D = b*b - 4*a*c
    assert(D>-0.0001)
    D = max(D, 0)
    t_max = (-b - D**0.5) / (2*a)
    return np.arctan2(dy + np.sin(angle)*t_max, dx + np.cos(angle)*t_max)

def get_arc(x, y, r, building):
    bx, by = building
    dx, dy = bx - x, by - y
    d = np.hypot(dx, dy)
    center_angle = np.arctan2(dy, dx)
    theta = np.arcsin(r / d)
    arc_start = center_angle - theta
    arc_end = center_angle + theta
    return normalize_arc(arc_start, arc_end)

def update_building_arc(unoccluded_segments, existing_arcs, x, y, building, r, theta):
    existing_arcs_copy = existing_arcs.copy()
    total = 0.0
    bx, by = building
    dy, dx = y - by, x - bx
    center_angle = np.arctan2(dy, dx)
    start_range = (center_angle - theta) % (2 * np.pi)
    end_range = (center_angle + theta) % (2 * np.pi)
    if(start_range < end_range):
        existing_arcs_copy.append((end_range, 2 * np.pi))
        existing_arcs_copy.append((0, start_range))
        extra = 2
    else:
        existing_arcs_copy.append((end_range, start_range))
        extra = 1
    for seg_start, seg_end in unoccluded_segments:
        start = get_angle(x, y, r, seg_end, building)
        end = get_angle(x, y, r, seg_start, building)
        new_arc = normalize_arc(start, end)
        new_ones = get_unoccluded_segments(new_arc, existing_arcs_copy)
        for s, e in new_ones:
            total += arc_diff(s, e) * r
        existing_arcs_copy = new_ones + existing_arcs_copy
    existing_arcs_copy = existing_arcs_copy[:-extra]
    dekha = False
    if total > 0:
        dekha = True
    return existing_arcs_copy, total, dekha
