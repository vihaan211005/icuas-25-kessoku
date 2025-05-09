import numpy as np

def is_valid(x, y, r, buildings, drones = 0, edge = 0):
    clear = (int)((drones)**0.5 + 2)*edge
    if(x - r < clear and y - r < clear):
        return False
    for bx, by in buildings:
        if np.hypot(bx - x, by - y) <= 2 * r:
            return False
    return True

def generate(num, r, world_width, world_height, drones, edge):
    buildings = []
    while len(buildings) < num:
        x = np.random.uniform(r, world_width - r)
        y = np.random.uniform(r, world_height - r)
        if is_valid(x, y, r, buildings, drones, edge):
            buildings.append((x, y))
    return buildings

def choose_observer(r, world_width, world_height, buildings):
    while True:
        obs_x = np.random.uniform(0, world_width)
        obs_y = np.random.uniform(0, world_height)
        if is_valid(obs_x, obs_y, r, buildings):
            return obs_x, obs_y

def building_info(building, obs_x, obs_y):
    bx, by = building
    dx, dy = bx - obs_x, by - obs_y
    return np.hypot(dx, dy)

def sort_buildings(buildings, obs_x, obs_y):
    return sorted(enumerate(buildings), key=lambda x: building_info(x[1], obs_x, obs_y))
