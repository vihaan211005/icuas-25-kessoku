import numpy as np
import json
import sys
import plot
import arc as arc_utils
import generator
import los
import gaussian

WORLD_WIDTH = 5.4
WORLD_HEIGHT = 7.2
BUILDING_RADIUS = 0.05
NUM_BUILDINGS = 7
OBS_RADIUS = 0.7
EDGE = 0.3
CLEARANCE = 0.3
NUM_DRONES = 4
M = (int)(WORLD_WIDTH / EDGE) + 1
N = (int)(WORLD_HEIGHT / EDGE) + 1
THETA = np.pi / 4
SIGMA = 2
np.random.seed(0)
VISIBLE_PENALTY = 1
RESET_TIMER = 10

buildings = generator.generate(NUM_BUILDINGS, BUILDING_RADIUS, WORLD_WIDTH, WORLD_HEIGHT, NUM_DRONES, EDGE)
visible_arcs_building = [[] for _ in range(NUM_BUILDINGS)]
matrix = los.create_matrix(WORLD_WIDTH, WORLD_HEIGHT, buildings, EDGE, BUILDING_RADIUS, CLEARANCE)
poses = [(0,0), (0,1), (1,0), (1,1)]
vis = np.zeros((M, N))

class Solution:
    def __init__(self):
        self.poses = []
        self.yaws = []
        self.matrix = None

solution = Solution()
solution.matrix = matrix.tolist()

def give_pose(state):
    new_poses = poses.copy()
    for i in range(NUM_DRONES):
        config = ((int)(state / pow(5, i)) % 5)
        if(config == 1):
            new_poses[i] = (poses[i][0], poses[i][1] + 1)
        elif(config == 2):
            new_poses[i] = (poses[i][0], poses[i][1] - 1)
        elif(config == 3):
            new_poses[i] = (poses[i][0] + 1, poses[i][1])
        elif(config == 4):
            new_poses[i] = (poses[i][0] - 1, poses[i][1])
        else:
            new_poses[i] = (poses[i][0], poses[i][1])
    return new_poses

def do_it():
    kitna_dekha = 0.0
    yaws = [[] for i in range(NUM_DRONES)]
    for i in range(NUM_DRONES):
        obs_x, obs_y = poses[i][0] * EDGE, poses[i][1] * EDGE
        obs_x_int, obs_y_int = poses[i]
        vis[obs_x_int][obs_y_int] -= VISIBLE_PENALTY
        sorted_buildings = generator.sort_buildings(buildings, obs_x, obs_y)
        
        visible_arcs = []

        for idx, (bx, by) in sorted_buildings:
            dx, dy = bx - obs_x, by - obs_y
            d = np.hypot(dx, dy)

            assert(d > BUILDING_RADIUS)

            if(d >= OBS_RADIUS):
                break

            arc = arc_utils.get_arc(obs_x, obs_y, BUILDING_RADIUS, (bx, by))
            unoccluded_segments = arc_utils.get_unoccluded_segments(arc, visible_arcs)
            visible_arcs += unoccluded_segments
            visible_arcs_building[idx], kitna, dekha = arc_utils.update_building_arc(unoccluded_segments, visible_arcs_building[idx], obs_x, obs_y, (bx, by), BUILDING_RADIUS, THETA)
            if dekha:
                kitna_dekha += kitna
                yaws[i].append(np.arctan2(dy, dx))
    solution.poses.append([(x * EDGE, y * EDGE) for (x, y) in poses])
    solution.yaws.append(yaws)
    return kitna_dekha

def main():
    if len(sys.argv) < 2:
        print("Usage: python main.py <EDGE>")
        sys.exit(1)

    EDGE = sys.argv[1]


counter = 0
''''PEHLA'''
do_it()
plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, poses, EDGE, matrix)
while(1):
    ''''CHOOSING'''
    counter += 1
    if(counter == 300):
        break
    print(counter)
    new_poses = poses.copy()

    prob_matrix = gaussian.make_matrix(buildings, EDGE, M, N, SIGMA, THETA, visible_arcs_building)
    metric = [-np.inf]

    for state in range(1, pow(5, NUM_DRONES)):
        total = 0.0
        new_poses = give_pose(state)
        
        ''''INVALID'''
        invalid_state = False
        for i in range(NUM_DRONES):
            if(new_poses[i][0] < 0 or new_poses[i][1] < 0 or new_poses[i][0] * EDGE > WORLD_WIDTH or new_poses[i][1] * EDGE > WORLD_HEIGHT):
                invalid_state = True
                break
            if(not matrix[new_poses[i][0]][new_poses[i][1]][new_poses[i][0]][new_poses[i][1]]):
                invalid_state = True
                break
            for j in range(i + 1, NUM_DRONES):
                if(new_poses[i] == new_poses[j]):
                    invalid_state = True
                    break
            if(invalid_state):
                break
        if(not invalid_state):
            in_los = [(0,0)]
            while(1):
                size1 = len(in_los)
                for i in in_los:
                    for j in new_poses:
                        if j not in in_los and matrix[i[0]][i[1]][j[0]][j[1]]:
                            in_los.append(j)
                if(size1 == len(in_los)):
                    break
            if(len(in_los) != NUM_DRONES + 1 and not ((0,0) in new_poses and len(in_los) == NUM_DRONES)):
                invalid_state = True
        
        if(invalid_state):
            metric.append(-np.inf)
            continue

        ''''UPDATING METRIC'''
        for i in range(NUM_DRONES):
            obs_x_int, obs_y_int = new_poses[i]

            total += vis[obs_x_int][obs_y_int]
            total += prob_matrix[obs_x_int][obs_y_int]
        metric.append(total)
    '''CHOOSE BEST'''
    state_new = np.argmax(metric)
    poses = give_pose(state_new)
    do_it()
    if(not (counter  % RESET_TIMER)):
        vis = np.zeros((M, N))
    if(not (counter % 100)):
        plot.stats(metric)
        plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, poses, EDGE, matrix)

data = json.dumps(solution.__dict__)
with open("solution.json", "w") as f:
    f.write(data)
