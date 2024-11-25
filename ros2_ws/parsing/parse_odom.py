def parse_odometry(file_path, output_path):
    positions = []  # Store (x, y) tuples
    in_position_section = False  # Track if we're in the pose->position section

    with open(file_path, 'r') as f:
        x, y = None, None  # Temporary storage for each position entry

        for line in f:
            # Strip any leading/trailing whitespace
            line = line.strip()

            # Detect entry into the pose->position section
            if line == "position:":
                in_position_section = True
                continue  # Move to the next line to read x, y values

            # Detect exit from the pose->position section
            if line.startswith("orientation:"):
                in_position_section = False

            # Extract x and y only if we're in the pose->position section
            if in_position_section:
                if line.startswith("x: ") and x is None:
                    x = float(line.split(": ")[1])
                elif line.startswith("y: ") and y is None:
                    y = float(line.split(": ")[1])

            # When both x and y values are captured, store them as a tuple
            if x is not None and y is not None:
                positions.append((x, y))
                # Reset x and y for the next entry
                x, y = None, None

    # Save the extracted positions to the output file
    with open(output_path, 'w') as out_f:
        for pos in positions:
            out_f.write(f"{pos[0]}, {pos[1]}\n")

    print(f"Extracted {len(positions)} positions saved to {output_path}")


# Usage example
input_file = 'ground_truth_odometry.txt'  # Replace with your file path
output_file = 'ground_truth_positions.csv'  # Output file for (x, y) data

parse_odometry(input_file, output_file)

