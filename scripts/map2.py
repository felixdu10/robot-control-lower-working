import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

# Load data
df = pd.read_csv('img.csv')

# Extract coordinates and values
x = df['x'].values
y = df['y'].values
z = df['val'].values

# Create grid to interpolate over
xi = np.linspace(x.min(), x.max(), 500)
yi = np.linspace(y.min(), y.max(), 500)
xi, yi = np.meshgrid(xi, yi)

# Interpolate data onto grid
zi = griddata((x, y), z, (xi, yi), method='linear')

# Set colormap with white for NaNs
cmap = plt.cm.hot
cmap.set_bad(color='white')

# Plot
plt.imshow(zi, extent=(x.min(), x.max(), y.min(), y.max()),
           origin='lower', cmap=cmap, aspect='auto')
cbar = plt.colorbar(label='maximum needle tip error(mm)')
cbar.set_label('maximum needle tip error(mm)', fontsize=20)
plt.xlabel('robot coordinate x (mm)', fontsize=20)
plt.ylabel('robot coordinate y (mm)', fontsize=20)
plt.title('Extended Upright Needle Tip Error by Location Based on a .03mm Uncertainty in Slider Postions', fontsize=30)
plt.show()