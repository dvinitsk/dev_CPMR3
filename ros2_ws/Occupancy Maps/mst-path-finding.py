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

def find_path_through_mst(points, mst_edges, start_idx, goal_idx):
    # Build adjacency list representation of MST
    graph = {i: [] for i in range(len(points))}
    for u, v, weight in mst_edges:
        graph[u].append((v, weight))
        graph[v].append((u, weight))
    
    # BFS to find path between start and goal
    visited = set()
    parent = {start_idx: None}
    queue = [start_idx]
    visited.add(start_idx)
    
    while queue:
        current = queue.pop(0)
        if current == goal_idx:
            break
        
        for neighbor, _ in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append(neighbor)
    
    # Reconstruct path
    path = []
    node = goal_idx
    while node is not None:
        path.append(points[node])
        node = parent.get(node)
    path.reverse()
    
    return path

def main():
    map_path = 'dilated_occupancy_map.png'
    num_samples = 500

    start_point = (449, 830)
    goal_point = (350, 268)

    # Load binary occupancy map
    binary_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    _, binary_map = cv2.threshold(binary_map, 127, 255, cv2.THRESH_BINARY)

    # Sample free space
    samples = sampling_of_free_space(binary_map, num_samples)
    samples.append(start_point)
    samples.append(goal_point)

    # Generate Minimum Spanning Tree
    mst_edges = kruskal_mst(samples, binary_map)

    # Find path through MST
    shortest_path = find_path_through_mst(samples, mst_edges, len(samples) - 2, len(samples) - 1)

    # Visualize result
    rrt_world = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
    
    # Draw MST edges
    for u, v, _ in mst_edges:
        pt1 = samples[u]
        pt2 = samples[v]
        cv2.line(rrt_world, pt1, pt2, (200, 200, 200), 1)  # Light gray edges

    # Draw shortest path
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            cv2.line(rrt_world, shortest_path[i], shortest_path[i + 1], (0, 255, 0), 2)  # Green path

    cv2.imshow("Path through Minimum Spanning Tree", rrt_world)
    cv2.imwrite("mst_path.png", rrt_world)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
