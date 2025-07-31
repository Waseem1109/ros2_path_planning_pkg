#!/usr/bin/env python3
"""
Generate PGM obstacle maps for path planning testing
"""

import numpy as np
import os

def save_pgm(grid, filename, comment="Generated obstacle map"):
    """Save a numpy array as PGM file"""
    height, width = grid.shape
    
    with open(filename, 'wb') as f:
        # PGM header
        f.write(b'P5\n')
        f.write(f'# {comment}\n'.encode())
        f.write(f'{width} {height}\n'.encode())
        f.write(b'255\n')
        
        # Write image data
        f.write(grid.astype(np.uint8).tobytes())
    
    print(f"Saved {filename} ({width}x{height})")

def create_corridor_map():
    """Map with narrow corridors - tests path finding through tight spaces"""
    grid = np.full((200, 200), 255, dtype=np.uint8)  # 10m x 10m at 0.05 resolution
    
    # Horizontal walls
    grid[60:70, :] = 0    # Wall across middle
    grid[130:140, :] = 0  # Wall in upper section
    
    # Vertical walls  
    grid[:, 60:70] = 0    # Wall down middle
    grid[:, 130:140] = 0  # Wall on right side
    
    # Create openings in walls
    grid[60:70, 20:30] = 255   # Gap in horizontal wall
    grid[60:70, 170:180] = 255 # Another gap
    grid[130:140, 80:90] = 255 # Gap in upper wall
    grid[20:30, 60:70] = 255   # Gap in vertical wall
    grid[170:180, 60:70] = 255 # Another gap
    
    save_pgm(grid, "corridor_map.pgm", "Narrow corridor maze")
    
    # Test scenarios
    scenarios = [
        {"name": "Short path", "start": (0.5, 0.5), "goal": (2.0, 2.0)},
        {"name": "Cross corridor", "start": (0.5, 0.5), "goal": (8.5, 0.5)},
        {"name": "Multi-turn", "start": (0.5, 0.5), "goal": (8.5, 8.5)},
        {"name": "Reverse path", "start": (8.5, 8.5), "goal": (0.5, 0.5)}
    ]
    
    return scenarios

def create_rooms_map():
    """Map with rooms connected by doorways - tests strategic path planning"""
    grid = np.full((160, 200), 255, dtype=np.uint8)  # 8m x 10m
    
    # Room boundaries
    # Room 1 (bottom-left)
    grid[0:80, 0:2] = 0      # Left wall
    grid[0:2, 0:80] = 0      # Bottom wall  
    grid[78:80, 0:80] = 0    # Top wall
    grid[0:80, 78:80] = 0    # Right wall
    
    # Room 2 (bottom-right)
    grid[0:80, 120:200] = 0  # Fill room area first
    grid[20:60, 140:180] = 255  # Clear interior
    
    # Room 3 (top-left)
    grid[80:160, 0:80] = 0   # Fill area
    grid[100:140, 20:60] = 255  # Clear interior
    
    # Room 4 (top-right)  
    grid[80:160, 120:200] = 0  # Fill area
    grid[100:140, 140:180] = 255  # Clear interior
    
    # Hallway connecting rooms
    grid[35:45, 80:120] = 255   # Horizontal hallway
    grid[80:100, 35:45] = 255   # Vertical connector
    grid[80:100, 135:145] = 255 # Another vertical connector
    
    # Doors between rooms
    grid[35:45, 78:82] = 255    # Door room 1 to hallway
    grid[35:45, 118:122] = 255  # Door hallway to room 2
    grid[78:82, 35:45] = 255    # Door room 1 to room 3
    grid[98:102, 135:145] = 255 # Door room 3 to room 4
    
    save_pgm(grid, "rooms_map.pgm", "Connected rooms layout")
    
    scenarios = [
        {"name": "Same room", "start": (1.0, 1.0), "goal": (3.0, 3.0)},
        {"name": "Adjacent rooms", "start": (1.0, 1.0), "goal": (1.0, 6.0)},
        {"name": "Diagonal rooms", "start": (1.0, 1.0), "goal": (8.0, 6.0)},
        {"name": "Far rooms", "start": (1.0, 1.0), "goal": (8.0, 8.0)}
    ]
    
    return scenarios

def create_obstacles_map():
    """Map with scattered circular and rectangular obstacles"""
    grid = np.full((200, 200), 255, dtype=np.uint8)  # 10m x 10m
    
    # Large rectangular obstacles
    grid[40:80, 60:100] = 0    # Large block in center-left
    grid[120:160, 120:180] = 0 # Large block in upper-right
    grid[20:40, 150:190] = 0   # Block in upper-left
    
    # Circular obstacles (approximate with squares)
    def add_circle(cx, cy, radius):
        for y in range(max(0, cy-radius), min(200, cy+radius+1)):
            for x in range(max(0, cx-radius), min(200, cx+radius+1)):
                if (x-cx)**2 + (y-cy)**2 <= radius**2:
                    grid[y, x] = 0
    
    add_circle(160, 40, 15)   # Circle in bottom-right
    add_circle(60, 140, 12)   # Circle in upper-left  
    add_circle(180, 180, 10)  # Small circle in corner
    add_circle(30, 30, 8)     # Small circle bottom-left
    
    # Linear obstacles (walls)
    grid[100:110, 20:60] = 0     # Horizontal wall
    grid[140:180, 95:105] = 0    # Vertical wall section
    
    save_pgm(grid, "obstacles_map.pgm", "Scattered obstacles")
    
    scenarios = [
        {"name": "Simple path", "start": (0.5, 0.5), "goal": (2.0, 2.0)},
        {"name": "Around obstacles", "start": (0.5, 0.5), "goal": (9.0, 9.0)},
        {"name": "Between circles", "start": (1.0, 7.0), "goal": (8.0, 2.0)},
        {"name": "Tight squeeze", "start": (1.0, 5.0), "goal": (9.0, 5.0)}
    ]
    
    return scenarios

def create_maze_map():
    """Classic maze layout - tests complex pathfinding"""
    grid = np.full((200, 200), 0, dtype=np.uint8)  # Start with all obstacles
    
    # Create maze paths (white = free space)
    # Main corridors
    grid[10:190, 10:20] = 255    # Left corridor
    grid[10:190, 180:190] = 255  # Right corridor  
    grid[10:20, 10:190] = 255    # Bottom corridor
    grid[180:190, 10:190] = 255  # Top corridor
    
    # Internal maze paths
    grid[10:50, 40:50] = 255     # Bottom section
    grid[40:60, 10:80] = 255     # Connector
    grid[50:90, 60:70] = 255     # Middle section
    grid[80:120, 40:110] = 255   # Central area
    grid[110:150, 80:90] = 255   # Upper-middle
    grid[140:180, 50:120] = 255  # Upper section
    grid[100:140, 130:140] = 255 # Top connector
    grid[60:100, 150:160] = 255  # Upper path
    
    # Cross connections
    grid[80:90, 80:120] = 255    # Horizontal cross
    grid[120:140, 100:110] = 255 # Another cross
    
    save_pgm(grid, "maze_map.pgm", "Complex maze layout")
    
    scenarios = [
        {"name": "Short maze", "start": (0.5, 0.5), "goal": (2.0, 0.5)},
        {"name": "Medium path", "start": (0.5, 0.5), "goal": (5.0, 3.0)}, 
        {"name": "Long path", "start": (0.5, 0.5), "goal": (9.0, 9.0)},
        {"name": "Dead end test", "start": (0.5, 0.5), "goal": (7.0, 2.0)}
    ]
    
    return scenarios

def create_spiral_map():
    """Spiral obstacle pattern - tests heuristic performance"""
    grid = np.full((200, 200), 255, dtype=np.uint8)
    
    # Create spiral obstacle
    cx, cy = 100, 100  # Center
    
    # Spiral parameters
    for angle in np.linspace(0, 6*np.pi, 1000):
        radius = 5 + angle * 3  # Growing radius
        x = int(cx + radius * np.cos(angle))
        y = int(cy + radius * np.sin(angle))
        
        if 0 <= x < 200 and 0 <= y < 200:
            # Create thick obstacle line
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < 200 and 0 <= ny < 200:
                        grid[ny, nx] = 0
    
    # Clear start and goal areas
    grid[0:20, 0:20] = 255      # Start area
    grid[180:200, 180:200] = 255 # Goal area
    
    save_pgm(grid, "spiral_map.pgm", "Spiral obstacle pattern")
    
    scenarios = [
        {"name": "Spiral challenge", "start": (0.5, 0.5), "goal": (9.5, 9.5)},
        {"name": "Partial spiral", "start": (0.5, 0.5), "goal": (5.0, 5.0)},
        {"name": "Reverse spiral", "start": (9.5, 9.5), "goal": (0.5, 0.5)},
        {"name": "Cross spiral", "start": (0.5, 9.5), "goal": (9.5, 0.5)}
    ]
    
    return scenarios

def main():
    """Generate all maps and print usage instructions"""
    print("Generating obstacle maps for path planning...")
    
    # Generate all maps directly in current directory
    all_scenarios = {
        "corridor_map.pgm": create_corridor_map(),
        "rooms_map.pgm": create_rooms_map(), 
        "obstacles_map.pgm": create_obstacles_map(),
        "maze_map.pgm": create_maze_map(),
        "spiral_map.pgm": create_spiral_map()
    }
    
    print("\n" + "="*60)
    print("GENERATED MAPS WITH TEST SCENARIOS")
    print("="*60)
    
    for map_file, scenarios in all_scenarios.items():
        print(f"\n📍 {map_file.upper()}")
        print("-" * 40)
        for scenario in scenarios:
            start = scenario['start']
            goal = scenario['goal']
            print(f"{scenario['name']:15} | Start: {start} | Goal: {goal}")
    
    print("\n" + "="*60)
    print("USAGE INSTRUCTIONS")
    print("="*60)
    print("1. Maps generated in current directory")
    print("2. Update your launch file parameters:")
    print("   - map_file: 'maps/corridor_map.pgm'")
    print("   - start_x: 0.5, start_y: 0.5")
    print("   - goal_x: 2.0, goal_y: 2.0")
    print("3. Run your path planner and test different scenarios")
    print("4. Try different circle_radius values for each map")
    print("\nNote: All coordinates assume 0.05m resolution, 10x10m maps")

if __name__ == "__main__":
    main()