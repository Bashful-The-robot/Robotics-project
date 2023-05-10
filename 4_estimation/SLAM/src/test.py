#%%
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import binary_dilation, gaussian_filter
from scipy.ndimage.morphology import generate_binary_structure, binary_erosion

# Create a sample binary occupancy grid
occupancy_grid = np.zeros((100, 100), dtype=bool)
occupancy_grid[50, 50] = True

radius = 6 # 5cm per square -> 30cm radium
        
y,x = np.ogrid[-8:9,-8:9]
mask = x**2 + y**2 <=radius**2 # boolean array

circle = np.zeros((17,17))
circle[mask] = 100
#self.circle += 100

# Inflate the grid using circular binary dilation
inflated_grid = binary_dilation(occupancy_grid, structure=circle)

# Apply a Gaussian filter to the inflated grid
sigma = 5
gaussian_grid = gaussian_filter(inflated_grid.astype(float), sigma=sigma)

# Normalize the values to range from 0 to 1
gaussian_grid = gaussian_grid / np.max(gaussian_grid)

# Set the mean value to a high value
mean_value = 80
gaussian_grid = gaussian_grid * mean_value + (1 - gaussian_grid) * 10

# Cap the maximum value to 101
gaussian_grid = np.clip(gaussian_grid, 0, 101)


plt.imshow(gaussian_grid)