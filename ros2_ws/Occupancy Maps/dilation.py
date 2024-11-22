import numpy as np
import cv2
import matplotlib.pyplot as plt

img_path = 'white_walls_occupancy_map.png'
binary_map = cv2.imread(img_path)
# Define the robot's radius in pixels (convert from meters if needed)
robot_diagonal_distance = ((0.4 ** 2) + (0.36 ** 2)) ** 0.5
robot_radius_meters = robot_diagonal_distance / 2
robot_radius_pixels = 20 * robot_radius_meters  # 10px/m *  

kernel_size = int(robot_radius_pixels)
kernel_size = max(1, kernel_size)

# Create a circular kernel for dilation
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * kernel_size, 2 * kernel_size))

# Dilate the map
dilated_map = cv2.dilate(binary_map, kernel)

# Display the dilated occupancy map
plt.imshow(dilated_map, cmap='gray')
plt.title("Dilated Occupancy Map")
plt.show()

# Save the dilated occupancy map
cv2.imwrite('dilated_occupancy_map.png', dilated_map)
print("Dilated occupancy map saved as dilated_occupancy_map.png")

