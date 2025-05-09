import numpy as np
import json
import sys
import plot
import arc as arc_utils
import generator
import los
import gaussian
import os

class Solution:
    def __init__(self):
        self.poses = []
        self.yaws = []
        self.matrix = None
        self.min_bound = None

def give_pose(state, NUM_DRONES, poses):
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

def do_it(THETA, OBS_RADIUS, VISIBLE_PENALTY, NUM_DRONES, EDGE, poses, vis, buildings, BUILDING_RADIUS, visible_arcs_building, solution):
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
    solution.poses.append(poses)
    solution.yaws.append(yaws)
    return kitna_dekha

def main():
    WORLD_WIDTH = 7.2
    WORLD_HEIGHT = 5.4
    BUILDING_RADIUS = 0.05
    NUM_BUILDINGS = None
    OBS_RADIUS = 0.7
    EDGE = 0.3
    CLEARANCE = 0.3
    NUM_DRONES = 4
    THETA = np.pi / 4
    SIGMA = 2
    np.random.seed(0)
    VISIBLE_PENALTY = 1
    RESET_TIMER = 10
    MIN_X = 0.0
    MIN_Y = 0.0
    COMM_RANGE = 2.5
    M = None
    N = None
    
    buildings = None
    visible_arcs_building = None
    matrix = None
    poses = [(0,0), (0,1), (1,0), (1,1)]
    vis = None
    
    '''TAKING INPUT'''
    if len(sys.argv) < 3:
        print("Usage: python main.py <EDGE> <OCTOMAP.JSON>")
        sys.exit(1)

    EDGE = float(sys.argv[1])
    json_path = sys.argv[2]
    
    '''SHELL VARS'''
    COMM_RANGE = float(os.getenv("COMM_RANGE", COMM_RANGE))
    NUM_DRONES = int(os.getenv("NUM_ROBOTS", NUM_DRONES))
    '''OCTOMAP JSON'''
    with open(json_path, 'r') as f:
        data = json.load(f)
    min_bound = data.get("min_bound", [MIN_X, MIN_Y, 0])
    MIN_X, MIN_Y = float(min_bound[0]), float(min_bound[1])
    circles = data.get("circles")
    circles = [i["real_world_center"] for i in circles]
    buildings = [(x[0] - MIN_X, x[1] - MIN_Y) for x in circles]

    '''UPDATING'''
    M = (int)(WORLD_WIDTH / EDGE) + 1
    N = (int)(WORLD_HEIGHT / EDGE) + 1
    NUM_BUILDINGS = len(buildings)
    visible_arcs_building = [[] for i in range(NUM_BUILDINGS)]
    matrix = los.create_matrix(WORLD_WIDTH, WORLD_HEIGHT, buildings, EDGE, BUILDING_RADIUS, CLEARANCE, COMM_RANGE)
    poses = poses[:NUM_DRONES]
    vis = np.zeros((M, N))
    solution = Solution()
    solution.matrix = matrix.tolist()
    solution.min_bound = [MIN_X, MIN_Y]

    counter = 0
    ''''PEHLA'''
    do_it(THETA, OBS_RADIUS, VISIBLE_PENALTY, NUM_DRONES, EDGE, poses, vis, buildings, BUILDING_RADIUS, visible_arcs_building, solution)
    # plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, poses, EDGE, matrix)
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
            new_poses = give_pose(state, NUM_DRONES, poses)
            
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
        poses = give_pose(state_new, NUM_DRONES, poses)
        do_it(THETA, OBS_RADIUS, VISIBLE_PENALTY, NUM_DRONES, EDGE, poses, vis, buildings, BUILDING_RADIUS, visible_arcs_building, solution)
        if(not (counter  % RESET_TIMER)):
            vis = np.zeros((M, N))
        if(not (counter % 100)):
            # plot.stats(metric)
            # plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, poses, EDGE, matrix)

    data = json.dumps(solution.__dict__)
    with open("/solution.json", "w") as f:
        f.write(data)

if(__name__ == '__main__'):
    main()