#!/usr/bin/env python3
"""
Create a simple test map with guaranteed free space
"""

import numpy as np

# Create 200x200 map (10m x 10m at 0.05 resolution)
grid = np.full((200, 200), 255, dtype=np.uint8)  # All free space (white)

# Add just a few simple obstacles to test
grid[80:120, 80:120] = 0    # Central square obstacle
grid[150:170, 50:70] = 0    # Small rectangle

# Ensure corners are definitely free
grid[0:30, 0:30] = 255      # Bottom-left corner free
grid[170:200, 170:200] = 255 # Top-right corner free
grid[0:30, 170:200] = 255   # Top-left corner free  
grid[170:200, 0:30] = 255   # Bottom-right corner free

# Save as PGM
with open("simple_test.pgm", 'wb') as f:
    f.write(b'P5\n')
    f.write(b'# Simple test map with guaranteed free space\n')
    f.write(b'200 200\n')
    f.write(b'255\n')
    f.write(grid.tobytes())

print("Created simple_test.pgm")
print("Test with:")
print("  start_x:=0.5 start_y:=0.5 goal_x:=9.5 goal_y:=9.5")
print("  (This should be grid (10,10) to (190,190) - definitely free)")