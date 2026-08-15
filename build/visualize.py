import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import math

# --- CONFIGURATION ---
TELEMETRY_FILE = "telemetry.csv"
NUM_RAYS = 64
MAX_LIDAR_RANGE = 20.0

# Map Bounds
BOUNDS = {
    'x_min': -500.0, 'x_max': 500.0,
    'y_min': 0.0,   'y_max': 100.0, # Y is Altitude
    'z_min': -500.0, 'z_max': 500.0
}

# Obstacles: (x, y, z, radius) - Now centered on the sea floor (Y=0)
OBSTACLES = [
    (15.0, 0.0, 15.0, 12.0)
]

def load_telemetry(filename):
    times, xs, ys, zs = [], [], [], []
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row['time']))
                xs.append(float(row['x']))
                ys.append(float(row['y']))
                zs.append(float(row['z']))
    except FileNotFoundError:
        print(f"Error: {filename} not found.")
        exit(1)
    return times, xs, ys, zs

def get_fibonacci_sphere(num_points):
    rays = []
    phi = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(num_points):
        y = 1.0 - (i / float(num_points - 1)) * 2.0
        radius = math.sqrt(1.0 - y * y)
        theta = phi * i
        x = math.cos(theta) * radius
        z = math.sin(theta) * radius
        rays.append(np.array([x, y, z]))
    return rays

def draw_hemisphere(ax, x_center, y_center, z_center, radius, color='r', alpha=0.4):
    # u: azimuth angle (0 to 2pi), v: elevation angle (0 to pi/2 for half sphere)
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi / 2, 15) 
    
    x = x_center + radius * np.outer(np.cos(u), np.sin(v))
    z = z_center + radius * np.outer(np.sin(u), np.sin(v))
    y = y_center + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    
    # Plotting (X, Z, Y) because Matplotlib uses the 3rd argument as the vertical axis
    ax.plot_surface(x, z, y, color=color, alpha=alpha, edgecolor='k', linewidth=0.2)

def compute_ray_hit(origin, direction):
    min_dist = MAX_LIDAR_RANGE
    
    # 1. Check Obstacles
    for ox, oy, oz, radius in OBSTACLES:
        center = np.array([ox, oy, oz])
        
        # Internal collision check
        L_vec = center - origin
        if np.dot(L_vec, L_vec) <= radius * radius:
            return 0.0

        tca = np.dot(L_vec, direction)
        if tca < 0: continue
        d2 = np.dot(L_vec, L_vec) - tca * tca
        radius2 = radius * radius
        if d2 > radius2: continue
        
        thc = math.sqrt(radius2 - d2)
        hit_dist = tca - thc
        if 0 < hit_dist < min_dist:
            min_dist = hit_dist

    # 2. Check Sea Floor (Y = 0)
    if direction[1] < 0:
        t_floor = (0.0 - origin[1]) / direction[1]
        if 0 < t_floor < min_dist:
            min_dist = t_floor

    return min_dist

def main():
    times, xs, ys, zs = load_telemetry(TELEMETRY_FILE)
    rays_dirs = get_fibonacci_sphere(NUM_RAYS)
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Draw Trajectory
    ax.plot(xs, zs, ys, color='blue', linewidth=1, alpha=0.5)
    
    # Draw Sea Floor (Grid)
    xx, zz = np.meshgrid(np.linspace(-30, 40, 5), np.linspace(-30, 30, 5))
    yy = np.zeros_like(xx)
    ax.plot_surface(xx, zz, yy, color='blue', alpha=0.1)

    # Draw Hemispheres
    for obs in OBSTACLES:
        draw_hemisphere(ax, obs[0], obs[1], obs[2], obs[3], color='darkorange')

    # Prepare objects for animation
    drone_marker, = ax.plot([], [], [], 'go', markersize=8)
    
    # Create 64 line objects for the Lidar rays
    ray_lines = []
    for _ in range(NUM_RAYS):
        line, = ax.plot([], [], [], color='cyan', alpha=0.3, linewidth=1)
        ray_lines.append(line)

    # Aesthetics
    ax.set_xlabel('X (Horizontal) [m]')
    ax.set_ylabel('Z (Depth) [m]')
    ax.set_zlabel('Y (Altitude) [m]')
    ax.set_xlim([-10, 40])
    ax.set_ylim([-20, 20])
    ax.set_zlim([0, 30])
    
    # Animation Update Function
    def update(frame):
        pos = np.array([xs[frame], ys[frame], zs[frame]])
        
        # Update drone marker
        drone_marker.set_data([pos[0]], [pos[2]])
        drone_marker.set_3d_properties([pos[1]])
        
        # Update each ray
        for i, ray_dir in enumerate(rays_dirs):
            hit_dist = compute_ray_hit(pos, ray_dir)
            end_pos = pos + ray_dir * hit_dist
            
            # Map coordinates to matplotlib 3D (X, Z, Y)
            ray_lines[i].set_data([pos[0], end_pos[0]], [pos[2], end_pos[2]])
            ray_lines[i].set_3d_properties([pos[1], end_pos[1]])
            
            # Change color if hitting an obstacle or the floor
            if hit_dist < MAX_LIDAR_RANGE:
                ray_lines[i].set_color('red')
                ray_lines[i].set_alpha(0.8)
            else:
                ray_lines[i].set_color('cyan')
                ray_lines[i].set_alpha(0.2)
                
        ax.set_title(f"Underwater Drone Simulation | T = {times[frame]:.2f}s")
        return [drone_marker] + ray_lines

    # Skip some frames (e.g., step=4) to make the animation play faster
    ani = FuncAnimation(fig, update, frames=range(0, len(xs), 4), interval=50, blit=False)
    
    plt.show()

if __name__ == '__main__':
    main()