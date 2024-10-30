def parse_odometry(file_path, output_path):
    positions = []  # Store (x, y, z) tuples

    with open(file_path, 'r') as f:
        x, y, z = None, None, None  # Temporary storage for each position entry

        for line in f:
            # Strip any leading/trailing whitespace
            line = line.strip()

            # Extract x, y, z position values
            if line.startswith("x: ") and x is None:
                x = float(line.split(": ")[1])
            elif line.startswith("y: ") and y is None:
                y = float(line.split(": ")[1])
            elif line.startswith("z: ") and z is None:
                z = float(line.split(": ")[1])

            # When all x, y, z values are captured, store them as a tuple
            if x is not None and y is not None and z is not None:
                positions.append((x, y, z))
                # Reset x, y, z for the next entry
                x, y, z = None, None, None

    # Save the extracted positions to the output file
    with open(output_path, 'w') as out_f:
        for pos in positions:
            out_f.write(f"{pos[0]}, {pos[1]}, {pos[2]}\n")

    print(f"Extracted {len(positions)} positions saved to {output_path}")


# Usage example
input_file = 'ground_truth_odometry.txt'  # Replace with your file path
output_file = 'extracted_positions.txt'  # Output file for (x, y, z) data

parse_odometry(input_file, output_file)
