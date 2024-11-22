import cv2
import numpy as np
import random
from scipy.sparse import csr_matrix
from heapq import heappop, heappush

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def sampling_of_free_space(binary_map, num_samples):
    samples_list = []
    for _ in range(num_samples):
        x_location = random.randint(0, binary_map.shape[1] - 1)
        y_location = random.randint(0, binary_map.shape[0] - 1)
        if binary_map[y_location, x_location] == 0:  # Free space
            samples_list.append((x_location, y_location))
    return samples_list

def is_collision_free(point_1, point_2, binary_map):
    points_on_line = np.linspace(point_1, point_2, int(distance(point_1, point_2))).astype(int)
    for point in points_on_line:
        x, y = point
        if 0 <= x < binary_map.shape[1] and 0 <= y < binary_map.shape[0]:
            if binary_map[y, x] == 255:  # Obstacle
                return False
    return True

def kruskal_mst(points, binary_map):
    # Create all possible edges
    edges = []
    num_points = len(points)
    for i in range(num_points):
        for j in range(i + 1, num_points):
            if is_collision_free(points[i], points[j], binary_map):
                edges.append((distance(points[i], points[j]), i, j))
    
    # Sort edges by weight
    edges.sort()
    
    # Disjoint Set Union (Union-Find) for tracking connectivity
    parent = list(range(num_points))
    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]
    
    def union(x, y):
        parent[find(x)] = find(y)
    
    # Kruskal's algorithm to build MST
    mst_edges = []
    for weight, u, v in edges:
        if find(u) != find(v):
            union(u, v)
            mst_edges.append((u, v, weight))
    
    return mst_edges

def dijkstra_shortest_path(points, mst_edges, start_idx, goal_idx):
    # Build graph from MST edges
    graph = {i: [] for i in range(len(points))}
    for u, v, weight in mst_edges:
        graph[u].append((v, weight))
        graph[v].append((u, weight))
    
    # Dijkstra's algorithm
    pq = [(0, start_idx)]
    distances = {i: float('inf') for i in range(len(points))}
    distances[start_idx] = 0
    predecessors = {i: None for i in range(len(points))}

    while pq:
        current_distance, current_node = heappop(pq)
        
        if current_node == goal_idx:
            break

        # If we've found a longer path, skip
        if current_distance > distances[current_node]:
            continue

        for neighbor, neighbor_distance in graph[current_node]:
            distance_through_current = current_distance + neighbor_distance
            if distance_through_current < distances[neighbor]:
                distances[neighbor] = distance_through_current
                predecessors[neighbor] = current_node
                heappush(pq, (distance_through_current, neighbor))

    # Reconstruct path
    path = []
    node = goal_idx
    while node is not None:
        path.append(points[node])
        node = predecessors[node]
    path.reverse()

    return path

def main():
    map_path = 'dilated_occupancy_map.png'
    num_samples = 1000

    start_point = (449, 830)
    goal_point = (427, 471) #Updated this value to 

    # Load binary occupancy map
    binary_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    _, binary_map = cv2.threshold(binary_map, 127, 255, cv2.THRESH_BINARY)

    # Sample free space
    samples = sampling_of_free_space(binary_map, num_samples)
    samples.append(start_point)
    samples.append(goal_point)

    # Generate Minimum Spanning Tree
    mst_edges = kruskal_mst(samples, binary_map)

    # Find shortest path
    shortest_path = dijkstra_shortest_path(samples, mst_edges, len(samples) - 2, len(samples) - 1)

    # Visualize result
    rrt_world = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
    
    # Draw MST edges in orange
    for u, v, _ in mst_edges:
        pt1 = samples[u]
        pt2 = samples[v]
        cv2.line(rrt_world, pt1, pt2, (0, 165, 255), 1)  # Orange edges (BGR color space)

    # Draw shortest path in magenta
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            cv2.line(rrt_world, shortest_path[i], shortest_path[i + 1], (255, 0, 255), 2)  # Magenta path

    # Print waypoints and display them as green circles
    print("Waypoints (from start to goal):")
    for waypoint in shortest_path:
        cv2.circle(rrt_world, waypoint, 5, (0, 255, 0), -1)  # Green circles
        print(waypoint)

    # Save waypoints to a file
    with open("waypoints.csv", "w") as file:
        for waypoint in shortest_path:
            file.write(f"{waypoint[0]},{waypoint[1]}\n")

    # Mark start and goal points
    # Start point: Blue circle
    cv2.circle(rrt_world, start_point, 8, (255, 0, 0), -1)  # Filled blue circle
    # Goal point: Red circle
    cv2.circle(rrt_world, goal_point, 8, (0, 0, 255), -1)  # Filled red circle

    cv2.imshow("Path through Minimum Spanning Tree", rrt_world)
    cv2.imwrite("mst_path_modified.png", rrt_world)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
