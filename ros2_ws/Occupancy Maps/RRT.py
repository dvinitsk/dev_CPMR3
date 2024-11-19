import cv2
import numpy as np
import math
import random
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from heapq import heappop, heappush


# Helper function to calculate Euclidean distance
def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


# Quadtree decomposition
class Quadtree:
    def __init__(self, x, y, width, height, threshold=1):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.threshold = threshold
        self.children = []
        self.homogeneous = False
        self.value = None

    def decompose(self, occupancy_map):
        region = occupancy_map[self.y:self.y + self.height, self.x:self.x + self.width]
        unique_values = np.unique(region)
        if len(unique_values) == 1:  # Homogeneous region
            self.homogeneous = True
            self.value = unique_values[0]
        elif min(self.width, self.height) > self.threshold:  # Split further
            half_width = self.width // 2
            half_height = self.height // 2
            self.children = [
                Quadtree(self.x, self.y, half_width, half_height, self.threshold),
                Quadtree(self.x + half_width, self.y, half_width, half_height, self.threshold),
                Quadtree(self.x, self.y + half_height, half_width, half_height, self.threshold),
                Quadtree(self.x + half_width, self.y + half_height, half_width, half_height, self.threshold)
            ]
            for child in self.children:
                child.decompose(occupancy_map)

    def is_collision_free(self, point_1, point_2, occupancy_map):
        points_on_line = np.linspace(point_1, point_2, int(distance(point_1, point_2))).astype(int)
        for point in points_on_line:
            x, y = point
            if 0 <= x < occupancy_map.shape[1] and 0 <= y < occupancy_map.shape[0]:
                if occupancy_map[y, x] == 255:  # Obstacle
                    return False
        return True


# Sampling free space
def sampling_of_free_space(binary_map, num_samples):
    samples_list = []
    for _ in range(num_samples):
        x_location = random.randint(0, binary_map.shape[1] - 1)
        y_location = random.randint(0, binary_map.shape[0] - 1)
        if binary_map[y_location, x_location] == 0:  # Free space
            samples_list.append((x_location, y_location))
    return samples_list


# Validate edges using the quadtree
def validate_edges(quadtree, points, binary_map):
    edges = []
    num_points = len(points)
    for i in range(num_points):
        for j in range(i + 1, num_points):
            if quadtree.is_collision_free(points[i], points[j], binary_map):
                edges.append((i, j, distance(points[i], points[j])))  # (start_idx, end_idx, weight)
    return edges


# Enforce global graph connectivity
def enforce_global_connectivity(points, edges, start_idx, goal_idx):
    num_points = len(points)
    graph = np.zeros((num_points, num_points))
    for edge in edges:
        i, j, weight = edge
        graph[i, j] = weight
        graph[j, i] = weight  # Undirected graph

    # Check connectivity
    num_components, labels = connected_components(csgraph=csr_matrix(graph), directed=False)
    print(f"Number of connected components: {num_components}")

    # If not fully connected, add edges to connect disjoint components
    if num_components > 1:
        for i in range(num_points):
            for j in range(i + 1, num_points):
                if labels[i] != labels[j]:  # Nodes in different components
                    edges.append((i, j, distance(points[i], points[j])))
                    graph[i, j] = distance(points[i], points[j])
                    graph[j, i] = distance(points[i], points[j])
                    labels[i] = labels[j]  # Merge components
                    break  # Add one edge at a time to connect components

    # Re-check connectivity
    num_components, labels = connected_components(csgraph=csr_matrix(graph), directed=False)
    print(f"Number of connected components after fixing: {num_components}")

    return edges


# Dijkstra's algorithm for shortest path
def dijkstra_shortest_path(points, edges, start_idx, goal_idx):
    graph = {i: [] for i in range(len(points))}
    for edge in edges:
        i, j, weight = edge
        graph[i].append((weight, j))
        graph[j].append((weight, i))  # Undirected graph

    # Dijkstra's algorithm
    pq = [(0, start_idx)]  # Priority queue (distance, node)
    distances = {i: float('inf') for i in range(len(points))}
    distances[start_idx] = 0
    predecessors = {i: None for i in range(len(points))}

    while pq:
        current_distance, current_node = heappop(pq)
        if current_node == goal_idx:
            break

        for neighbor_distance, neighbor in graph[current_node]:
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


# Main function
def main():
    map_path = 'dilated_occupancy_map.png'
    num_samples = 500

    start_point = (449, 830)
    goal_point = (350, 268)

    # Load binary occupancy map
    binary_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    _, binary_map = cv2.threshold(binary_map, 127, 255, cv2.THRESH_BINARY)

    # Build quadtree
    quadtree = Quadtree(0, 0, binary_map.shape[1], binary_map.shape[0], threshold=5)
    quadtree.decompose(binary_map)

    # Sample free space
    samples = sampling_of_free_space(binary_map, num_samples)
    samples.append(start_point)
    samples.append(goal_point)

    # Validate edges
    edges = validate_edges(quadtree, samples, binary_map)

    # Enforce global connectivity
    edges = enforce_global_connectivity(samples, edges, len(samples) - 2, len(samples) - 1)

    # Extract shortest path
    shortest_path = dijkstra_shortest_path(samples, edges, len(samples) - 2, len(samples) - 1)

    # Visualize result
    rrt_world = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
    for edge in edges:
        pt1 = samples[edge[0]]
        pt2 = samples[edge[1]]
        cv2.line(rrt_world, pt1, pt2, (200, 200, 200), 1)  # Light gray edges

    if shortest_path:
        for i in range(len(shortest_path) - 1):
            cv2.line(rrt_world, shortest_path[i], shortest_path[i + 1], (0, 255, 0), 2)  # Green shortest path

    cv2.imshow("Shortest Path", rrt_world)
    cv2.imwrite("shortest_path_with_quadtree_dijkstra.png", rrt_world)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

