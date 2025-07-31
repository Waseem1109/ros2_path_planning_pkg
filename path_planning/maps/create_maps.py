import numpy as np

# Create a blank 100x100 map
width, height = 100, 100
data = np.zeros((height, width), dtype=np.uint8)

# Write PGM file
with open('blank_map.pgm', 'wb') as f:
    f.write(b'P5\n')
    f.write(f'{width} {height}\n'.encode())
    f.write(b'255\n')
    f.write(data.tobytes())

print("Created blank_map.pgm")