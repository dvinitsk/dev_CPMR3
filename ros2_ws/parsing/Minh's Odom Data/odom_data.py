import pandas as pd
import matplotlib.pyplot as plt

# Load data
ground_truth = pd.read_csv('ground_truth_positions.csv', header=None, names=['x', 'y'])
stock_model = pd.read_csv('stock_model_positions.csv', header=None, names=['x', 'y'])
updated_model = pd.read_csv('updated_model_positions.csv', header=None, names=['x', 'y'])

# Plotting the x and y positions
plt.figure(figsize=(10, 8))

# Plot ground truth
plt.plot(ground_truth['x'], ground_truth['y'], label="Ground Truth", color="blue", linestyle='--')

# Plot stock model
plt.plot(stock_model['x'], stock_model['y'], label="Stock Model", color="red", alpha=0.7)

# Plot updated model
plt.plot(updated_model['x'], updated_model['y'], label="Updated Model", color="green", alpha=0.7)

# Labels and legend
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Comparison of X and Y Positions for Ground Truth, Stock Model, and Updated Model")
plt.legend()
plt.grid(True)

# Display plot
plt.show()

