def parse_pose_coordinates(input_file, output_file):
    positions = []  # Store (x, y, z) tuples

    with open(input_file, 'r') as f:
        capture_position = False  # Track if we're inside a 'position:' block
        x, y, z = None, None, None  # Temporary storage for each position entry

        for line in f:
            line = line.strip()  # Remove extra spaces

            # Detect the 'position:' block and start capturing coordinates
            if line == "position:":
                capture_position = True  # Start capturing the next 3 lines

            # Capture only when inside the 'position:' block
            if capture_position:
                if line.startswith("x:") and x is None:
                    x = float(line.split(": ")[1])
                elif line.startswith("y:") and y is None:
                    y = float(line.split(": ")[1])
                elif line.startswith("z:") and z is None:
                    z = float(line.split(": ")[1])

                # If all 3 coordinates are captured, store them and reset
                if x is not None and y is not None and z is not None:
                    positions.append((x, y, z))
                    x, y, z = None, None, None  # Reset for the next block
                    capture_position = False  # Stop capturing until next 'position:'

    # Write the extracted (x, y, z) coordinates to the output file
    with open(output_file, 'w') as out_f:
        for pos in positions:
            out_f.write(f"{pos[0]}, {pos[1]}, {pos[2]}\n")

    print(f"Extracted {len(positions)} (x, y, z) positions saved to {output_file}")


# Usage example
input_file = 'ground_truth_odometry.txt'  # Replace with your input file path
output_file = 'filtered_positions.txt'    # Replace with your desired output file path

# Run the function to extract the coordinates
parse_pose_coordinates(input_file, output_file)

