import numpy as np
import pandas as pd

def load_data(file_path):
    """
    Load the data from a CSV file without headers. Assumes columns for x, y, and z positions.
    """
    data = pd.read_csv(file_path, header=None)
    return data

def calculate_rmse(data1, data2):
    """
    Calculate the RMSE between two datasets for x and y positions.
    - data1: DataFrame containing ground truth x, y positions
    - data2: DataFrame containing model x, y positions
    """
    # Ensure both datasets have the same number of points
    if len(data1) != len(data2):
        raise ValueError("Datasets must have the same number of points")

    # Display the number of observations
    num_observations = len(data1)
    print(f"Number of observations: {num_observations}")

    # Calculate RMSE for x and y positions (using columns 0 and 1)
    rmse_x = np.sqrt(np.mean((data1.iloc[:, 0] - data2.iloc[:, 0]) ** 2))  # Column 0 for x
    rmse_y = np.sqrt(np.mean((data1.iloc[:, 1] - data2.iloc[:, 1]) ** 2))  # Column 1 for y
    
    return rmse_x, rmse_y

# Load the datasets (adjust file paths as necessary)
ground_truth = load_data('ground_truth_positions_truncated_x_y_adjusted.csv')
stock_model = load_data('filtered_positions_truncated_x_y.csv')

# Calculate RMSE for x and y
rmse_x, rmse_y = calculate_rmse(ground_truth, stock_model)

print(f"RMSE for x positions: {rmse_x:.3f}")
print(f"RMSE for y positions: {rmse_y:.3f}")
