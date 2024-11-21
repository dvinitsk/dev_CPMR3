import csv

def process_waypoints(input_file, output_file='waypoints_gazebo.csv', reference_x=449, reference_y=830, scale_factor=0.1):
    """
    Process waypoints from pixel coordinates to Gazebo coordinates and save to a file
    
    Args:
        input_file (str): Path to input CSV file
        output_file (str): Path to output CSV file
        reference_x (int): Reference x pixel coordinate of robot's initial position
        reference_y (int): Reference y pixel coordinate of robot's initial position
        scale_factor (float): Scale factor to convert pixels to meters
        
    Returns:
        list: List of tuples containing (x, y) coordinates in Gazebo
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
                        gazebo_x = (reference_y - pixel_y) * scale_factor
                        gazebo_y = (reference_x - pixel_x) * scale_factor
                        
                        processed_waypoints.append((gazebo_x, gazebo_y))
                    except ValueError:
                        print(f"Skipping invalid row: {row}")
                        continue
        
        # Save processed waypoints to output file
        with open(output_file, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(['Gazebo_X', 'Gazebo_Y'])  # Write header
            csv_writer.writerows(processed_waypoints)
        
        print(f"Processed waypoints saved to {output_file}")
        return processed_waypoints
    except FileNotFoundError:
        print(f"Error: Could not find file {input_file}")
        return []
    except Exception as e:
        print(f"Error processing waypoints: {e}")
        return []

if __name__ == '__main__':
    # Input file containing pixel coordinates
    input_file = 'waypoints.csv'
    output_file = 'waypoints_gazebo.csv'
    
    # Process waypoints
    waypoints = process_waypoints(input_file, output_file)
    print("Processed waypoints (in Gazebo coordinates):")
    for i, (x, y) in enumerate(waypoints):
        print(f"Waypoint {i+1}: ({x:.2f}, {y:.2f})")

