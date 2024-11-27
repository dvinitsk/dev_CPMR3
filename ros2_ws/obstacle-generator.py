import json
import random

def generate_random_obstacles(num_obstacles=20, world_size=1000, min_radius=30, max_radius=100):
    obstacles = []
    for _ in range(num_obstacles):
        radius = random.randint(min_radius, max_radius)
        x = random.randint(radius, world_size - radius)
        y = random.randint(radius, world_size - radius)
        
        obstacles.append({
            "x": x,
            "y": y,
            "radius": radius
        })
    
    obstacle_map = {"obstacles": obstacles}
    
    with open('obstacles.json', 'w') as f:
        json.dump(obstacle_map, f, indent=2)

if __name__ == "__main__":
    generate_random_obstacles()
