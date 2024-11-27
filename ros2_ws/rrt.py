import cv2
import numpy as np
import json
import random
import math
from typing import Dict, List, Tuple, Optional

class World:
    def __init__(self, size: int = 1000):
        self.size = size
        self.world_image = np.ones((size, size, 3), np.uint8) * 255
        self.obstacles = []
        
    @classmethod
    def from_json(cls, json_path: str, size: int = 1000) -> 'World':
        """Create world from JSON file"""
        world = cls(size)
        try:
            with open(json_path, 'r') as file:
                data = json.load(file)
                world.obstacles = data['obstacles']
                print(f"Loaded {len(world.obstacles)} obstacles")
        except Exception as e:
            print(f"Error loading obstacles: {e}")
            return None
        return world
    
    def draw_obstacles(self, image):
        """Draw obstacles on the specified image"""
        for obs in self.obstacles:
            x = int(obs['x'])
            y = int(obs['y'])
            r = int(obs['radius'])
            # Draw filled circle in blue
            cv2.circle(image, (x, y), r, (255, 0, 0), -1)
            # Draw circle border in darker blue
            cv2.circle(image, (x, y), r, (150, 0, 0), 2)

class RRTPlanner:
    def __init__(self, world: World):
        self.world = world
        self.samples: List[Tuple[int, int]] = []
        self.tree: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {}
    
    def display_samples(self, num_samples: int = 100) -> None:
        """Display sampling stage"""
        # Create fresh image with obstacles
        sampling_image = np.ones((self.world.size, self.world.size, 3), np.uint8) * 255
        self.world.draw_obstacles(sampling_image)
        
        print(f"Sampling {num_samples} points...")
        valid_count = 0
        invalid_count = 0
        
        for _ in range(num_samples):
            x = random.randint(0, self.world.size - 1)
            y = random.randint(0, self.world.size - 1)
            
            if not np.array_equal(sampling_image[y, x], [255, 0, 0]):  # Not in obstacle
                self.samples.append((x, y))
                cv2.circle(sampling_image, (x, y), 3, (0, 255, 0), -1)  # Valid points in green
                valid_count += 1
            else:
                cv2.circle(sampling_image, (x, y), 3, (0, 0, 255), -1)  # Invalid points in red
                invalid_count += 1
        
        print(f"Stage 1: Found {valid_count} valid and {invalid_count} invalid samples")
        cv2.imshow("Stage 1: Random Sampling", sampling_image)
        cv2.imwrite("stage1_random_sampling.png", sampling_image)
        cv2.waitKey(0)
    
    def display_tree_growth(self, start: Tuple[int, int]) -> None:
        """Display tree growth stage"""
        # Create fresh image with obstacles
        tree_image = np.ones((self.world.size, self.world.size, 3), np.uint8) * 255
        self.world.draw_obstacles(tree_image)
        
        print("Growing RRT tree...")
        self.tree = {start: None}
        # Draw root node
        cv2.circle(tree_image, start, 5, (0, 100, 0), -1)  # Dark green start point
        
        connections_made = 0
        for sample in self.samples:
            nearest = min(self.tree.keys(), 
                        key=lambda p: math.hypot(p[0] - sample[0], p[1] - sample[1]))
            
            # Check if path is clear
            points = np.linspace(nearest, sample, 
                               int(math.hypot(sample[0] - nearest[0], 
                                            sample[1] - nearest[1]))).astype(int)
            
            if not any(np.array_equal(tree_image[p[1], p[0]], [255, 0, 0]) for p in points):
                self.tree[sample] = nearest
                cv2.line(tree_image, nearest, sample, (0, 255, 0), 2)  # Green connections
                cv2.circle(tree_image, sample, 3, (0, 150, 0), -1)  # Tree nodes
                connections_made += 1
        
        print(f"Stage 2: Tree grown with {connections_made} successful connections")
        cv2.imshow("Stage 2: RRT Tree", tree_image)
        cv2.waitKey(0)
        cv2.imwrite("stage2_tree.png", tree_image)
        return tree_image
    
    def display_path(self, tree_image, start: Tuple[int, int], goal: Tuple[int, int]) -> None:
        """Display path finding stage"""
        # Create copy of tree image (which includes obstacles)
        path_image = tree_image.copy()
        
        print("Finding path to goal...")
        # Try to connect goal to tree
        if goal not in self.tree:
            nearest = min(self.tree.keys(), 
                        key=lambda p: math.hypot(p[0] - goal[0], p[1] - goal[1]))
            
            points = np.linspace(nearest, goal, 
                               int(math.hypot(goal[0] - nearest[0], 
                                            goal[1] - nearest[1]))).astype(int)
            
            if not any(np.array_equal(path_image[p[1], p[0]], [255, 0, 0]) for p in points):
                self.tree[goal] = nearest
                cv2.line(path_image, nearest, goal, (0, 255, 0), 2)
        
        # Find and draw path
        if goal in self.tree:
            path = []
            current = goal
            while current is not None:
                path.append(current)
                current = self.tree.get(current)
            
            # Draw path
            for i in range(len(path) - 1):
                cv2.line(path_image, path[i], path[i + 1], (255, 165, 0), 3)  # Orange path
            
            print(f"Stage 3: Found path with {len(path)} nodes")
        else:
            print("Stage 3: No path found")
        
        # Draw start and goal
        cv2.circle(path_image, start, 7, (0, 255, 0), -1)  # Green start
        cv2.circle(path_image, goal, 7, (0, 0, 255), -1)   # Red goal
        
        cv2.imshow("Stage 3: Path Finding", path_image)
        cv2.waitKey(0)
        cv2.imwrite("stage3_path.png", path_image)

def main():
    # Parameters
    MAP_SIZE = 1000
    START = (400, 100)
    GOAL = (900, 900)
    NUM_SAMPLES = 100
    
    # Create world and planner
    world = World.from_json('obstacles.json', MAP_SIZE)
    if world is None:
        print("Failed to load world")
        return
        
    planner = RRTPlanner(world)
    
    # Show each stage sequentially
    planner.display_samples(NUM_SAMPLES)
    tree_image = planner.display_tree_growth(START)
    planner.display_path(tree_image, START, GOAL)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
