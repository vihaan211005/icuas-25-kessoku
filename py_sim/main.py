import numpy as np
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
CLEARANCE = 0.1
NUM_DRONES = 4
M = (int)(WORLD_WIDTH / EDGE) + 1
N = (int)(WORLD_HEIGHT / EDGE) + 1
THETA = np.pi / 9
SIGMA = 2
np.random.seed(0)

buildings = generator.generate(NUM_BUILDINGS, BUILDING_RADIUS, WORLD_WIDTH, WORLD_HEIGHT, NUM_DRONES, EDGE)
visible_arcs_building = [[] for _ in range(NUM_BUILDINGS)]
matrix = los.create_matrix(WORLD_WIDTH, WORLD_HEIGHT, buildings, EDGE, BUILDING_RADIUS, CLEARANCE)
poses = [(0,0), (0,1), (1,0), (1,1)]
vis = np.zeros((M, N))

# false_count = np.sum(matrix == False)
# true_count = np.sum(matrix == True)
# print(false_count)
# print(true_count)

for i in range(NUM_DRONES):
    obs_x, obs_y = poses[i][0] * EDGE, poses[i][1] * EDGE
    obs_x_int, obs_y_int = poses[i]
    vis[obs_x_int][obs_y_int] -= 1e-4
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
        visible_arcs_building[idx], _ = arc_utils.update_building_arc(unoccluded_segments, visible_arcs_building[idx], obs_x, obs_y, (bx, by), BUILDING_RADIUS, THETA)
plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, poses, EDGE, matrix)

counter = 0
while(1):
    counter += 1
    print(counter)
    new_poses = poses.copy()

    metric = [-np.inf]

    prob_matrix = gaussian.make_matrix(buildings, EDGE, M, N, SIGMA, THETA, visible_arcs_building)
    metric2 = [-np.inf]

    for state in range(1, pow(5, NUM_DRONES)):
        visible_arcs_building_copy = visible_arcs_building.copy()
        total = 0.0
        total2 = 0.0

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
            metric2.append(-np.inf)
            continue


        
        for i in range(NUM_DRONES):
            obs_x, obs_y = new_poses[i][0] * EDGE, new_poses[i][1] * EDGE
            obs_x_int, obs_y_int = new_poses[i]

            total += vis[obs_x_int][obs_y_int]
            total2 += prob_matrix[obs_x_int][obs_y_int]
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
                visible_arcs_building_copy[idx], new = arc_utils.update_building_arc(unoccluded_segments, visible_arcs_building_copy[idx], obs_x, obs_y, (bx, by), BUILDING_RADIUS, THETA)
                total += new
        metric.append(total)
        metric2.append(total2)

    # plot.stats(metric)
    # plot.stats(metric2)
    max_val = np.max(metric)
    max_val2 = np.max(metric2)
    indices = np.where(metric == max_val)[0]
    indices2 = np.where(metric2 == max_val2)[0]

    # state_new = indices[0]
    state_new = indices2[0]
    for i in range(NUM_DRONES):
        config = ((int)(state_new / pow(5, i)) % 5)
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
    poses = new_poses

    for i in range(NUM_DRONES):
        obs_x, obs_y = poses[i][0] * EDGE, poses[i][1] * EDGE
        obs_x_int, obs_y_int = poses[i]
        vis[obs_x_int][obs_y_int] -= 1e-4
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
            visible_arcs_building[idx], _ = arc_utils.update_building_arc(unoccluded_segments, visible_arcs_building[idx], obs_x, obs_y, (bx, by), BUILDING_RADIUS, THETA)
    
    if(not (counter % 10)):
        plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, poses, EDGE, matrix)
    
# while(1):

#     obs_x, obs_y = generator.choose_observer(BUILDING_RADIUS, WORLD_WIDTH, WORLD_HEIGHT, buildings)
#     sorted_buildings = generator.sort_buildings(buildings, obs_x, obs_y)

#     visible_arcs = []

#     for idx, (bx, by) in sorted_buildings:
#         dx, dy = bx - obs_x, by - obs_y
#         d = np.hypot(dx, dy)

#         assert(d > BUILDING_RADIUS)

#         if(d >= OBS_RADIUS):
#             break

#         arc = arc_utils.get_arc(obs_x, obs_y, BUILDING_RADIUS, (bx, by))
#         unoccluded_segments = arc_utils.get_unoccluded_segments(arc, visible_arcs)
#         visible_arcs += unoccluded_segments
#         visible_arcs_building[idx], _ = arc_utils.update_building_arc(unoccluded_segments, visible_arcs_building[idx], obs_x, obs_y, (bx, by), BUILDING_RADIUS)

#     plot.plotter(WORLD_WIDTH, WORLD_HEIGHT, BUILDING_RADIUS, buildings, visible_arcs_building, [(obs_x / EDGE, obs_y / EDGE)], EDGE, matrix)
