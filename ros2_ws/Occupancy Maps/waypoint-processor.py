import csv
import numpy as np

def process_waypoints(input_file, reference_x=449, reference_y=830, scale_factor=1/10):
    """
    Process waypoints from pixel coordinates to robot pose coordinates
    
    Args:
        input_file (str): Path to input CSV file
        reference_x (int): Reference x pixel coordinate of robot's initial position
        reference_y (int): Reference y pixel coordinate of robot's initial position
        scale_factor (float): Scale factor to convert pixels to meters
        
    Returns:
        list: List of tuples containing (x, y) coordinates in meters
    """
    processed_waypoints = []
    
    try:
        with open(input_file, 'r') as f:
            csv_reader = csv.reader(f)
            for row in csv_reader:
                if len(row) >= 2:  # Ensure we have at least x,y coordinates
                    try:
                        pixel_x = float(row[0])
                        pixel_y = float(row[1])
                        
                        # Transform coordinates
                        pose_x = (pixel_x - reference_x) * scale_factor
                        pose_y = (pixel_y - reference_y) * scale_factor
                        
                        processed_waypoints.append((pose_x, pose_y))
                    except ValueError:
                        print(f"Skipping invalid row: {row}")
                        continue
                        
        return processed_waypoints
    except FileNotFoundError:
        print(f"Error: Could not find file {input_file}")
        return []
    except Exception as e:
        print(f"Error processing waypoints: {e}")
        return []

if __name__ == '__main__':
    # Example usage
    waypoints = process_waypoints('waypoints.csv')
    print("Processed waypoints (in meters):")
    for i, (x, y) in enumerate(waypoints):
        print(f"Waypoint {i+1}: ({x:.2f}, {y:.2f})")
