import cv2
import numpy as np
import json
import random
import math

def obstacles_load(path_for_json):
    with open(path_for_json, 'r') as file:
        obs_data = json.load(file)
    return obs_data['obstacles']


def generate_map_with_obstacles(circle_obstacles, map_width, map_height):
    generated_world = np.ones((map_height, map_width, 3), np.uint8) * 255  # Height first
    for obs in circle_obstacles:
        center_obs = (obs['x'], obs['y'])
        rad = obs['radius']
        cv2.circle(generated_world, center_obs, rad, (0, 0, 0), -1)
    return generated_world

def sampling_of_free_space(binary_map, num_samples):
    samples_list = []
    for _ in range(num_samples):
        x_location = random.randint(0, binary_map.shape[1] - 1)  # Width
        y_location = random.randint(0, binary_map.shape[0] - 1)  # Height
        
        if binary_map[y_location, x_location] == 0:
            samples_list.append((x_location, y_location))
    return samples_list

def locate_nearest(tree, created_point):
    return min(tree, key=lambda pt: math.hypot(created_point[0] - pt[0], created_point[1] - pt[1]))


def check_if_collision_free(binary_map, point_1, point_2):
    #Check if the path between two points is collision-free
    points_on_line = np.linspace(point_1, point_2, int(math.hypot(point_2[0] - point_1[0], point_2[1] - point_1[1]))).astype(int)
    for point in points_on_line:
        x, y = point
        if binary_map[y,x] == 255:
            return False
    return True


def grow_RRT_tree(generated_world, samples, start_point):
    tree = {start_point: None}
    for sample in samples:
        closest = locate_nearest(tree.keys(), sample)
        if check_if_collision_free(generated_world, closest, sample):
            tree[sample] = closest
            cv2.line(generated_world, closest, sample, (0, 0, 255), 1)
    return tree


def locate_shortest_path(tree, start_point, goal_point):
    shortest_path = []
    node = goal_point
    while node is not None:
        shortest_path.append(node)
        node = tree.get(node)
    shortest_path.reverse()
    return shortest_path if shortest_path[0] == start_point else []


def path_drawn(generated_world, shortest_path, color=(0, 255, 0)):
    for i in range(len(shortest_path) - 1):
        cv2.line(generated_world, shortest_path[i], shortest_path[i + 1], color, 2)


def load_occupancy_map(map_path):
    """
    Loads the occupancy map and converts it to a binary format.
    White (255): Walls/Obstacles
    Black (0): Free Space
    """
    occupancy_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    _, binary_map = cv2.threshold(occupancy_map, 127, 255, cv2.THRESH_BINARY)
    return binary_map


def main():
    map_path = 'dilated_occupancy_map.png'
    map_width = 1354 # Width in pixels
    map_height = 1048  # Height in pixels
    
    num_samples = 100
    resolution = 10 #pixels per meter

    start_point = (449, 830)
    goal_point = (350, 268) 

    binary_map = load_occupancy_map(map_path)

    samples = sampling_of_free_space(binary_map, num_samples)

    rrt_world = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
    cv2.circle(rrt_world, start_point, 5, (255, 0, 0), -1)
    cv2.circle(rrt_world, goal_point, 5, (0, 0, 255), -1)

    tree = grow_RRT_tree(binary_map, samples, start_point)

    if goal_point not in tree:
        closest_to_goal = locate_nearest(tree.keys(), goal_point)
        if check_if_collision_free(binary_map, closest_to_goal, goal_point):
            tree[goal_point] = closest_to_goal
            cv2.line(rrt_world, closest_to_goal, goal_point, (255, 0, 0), 1)

    shortest_path = locate_shortest_path(tree, start_point, goal_point)
    if shortest_path:
        path_drawn(rrt_world, shortest_path, (0, 255, 0))

    #Show and save the result
    cv2.imshow("RRT Path", rrt_world)
    cv2.imwrite("rrt_path_with_map.png", rrt_world)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
