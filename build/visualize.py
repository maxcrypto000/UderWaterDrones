import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import math

# --- CONFIGURATION ---
TELEMETRY_FILE = "telemetry.csv"
ENVIRONMENT_FILE = "environment.csv"
NUM_RAYS = 64
MAX_LIDAR_RANGE = 20.0

# Will be populated dynamically
BOUNDS = {}
OBSTACLES = []

def load_environment(filename):
    global BOUNDS, OBSTACLES
    try:
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if parts[0] == 'BOUNDS':
                    BOUNDS['x_min'] = float(parts[1])
                    BOUNDS['x_max'] = float(parts[2])
                    BOUNDS['y_min'] = float(parts[3])
                    BOUNDS['y_max'] = float(parts[4])
                    BOUNDS['z_min'] = float(parts[5])
                    BOUNDS['z_max'] = float(parts[6])
                elif parts[0] == 'OBS':
                    # x, y, z, radius
                    OBSTACLES.append((float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])))
    except FileNotFoundError:
        print(f"Error: {filename} not found. Did you run the C simulation first?")
        exit(1)

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

def draw_hemisphere(ax, x_center, y_center, z_center, radius, color='darkorange', alpha=0.4):
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi / 2, 15) 
    x = x_center + radius * np.outer(np.cos(u), np.sin(v))
    z = z_center + radius * np.outer(np.sin(u), np.sin(v))
    y = y_center + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, z, y, color=color, alpha=alpha, edgecolor='k', linewidth=0.2)

def compute_ray_hit(origin, direction):
    min_dist = MAX_LIDAR_RANGE
    for ox, oy, oz, radius in OBSTACLES:
        center = np.array([ox, oy, oz])
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
    if direction[1] < 0:
        t_floor = (0.0 - origin[1]) / direction[1]
        if 0 < t_floor < min_dist:
            min_dist = t_floor
    return min_dist

def main():
    load_environment(ENVIRONMENT_FILE)
    times, xs, ys, zs = load_telemetry(TELEMETRY_FILE)
    rays_dirs = get_fibonacci_sphere(NUM_RAYS)
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Draw Trajectory
    ax.plot(xs, zs, ys, color='blue', linewidth=1, alpha=0.5, label='Trajectory')
    
    # Draw Start Position (Star Marker)
    ax.scatter(xs[0], zs[0], ys[0], color='lime', s=200, marker='*', edgecolor='black', zorder=5, label='Start Position')
    
    # Draw Sea Floor using dynamic bounds
    xx, zz = np.meshgrid(np.linspace(BOUNDS['x_min'], BOUNDS['x_max'], 5), 
                         np.linspace(BOUNDS['z_min'], BOUNDS['z_max'], 5))
    yy = np.zeros_like(xx)
    ax.plot_surface(xx, zz, yy, color='blue', alpha=0.1)

    # Draw Dynamic Obstacles
    for obs in OBSTACLES:
        draw_hemisphere(ax, obs[0], obs[1], obs[2], obs[3])

    drone_marker, = ax.plot([], [], [], 'go', markersize=8, label='Drone')
    ray_lines = [ax.plot([], [], [], color='cyan', alpha=0.3, linewidth=1)[0] for _ in range(NUM_RAYS)]

    ax.set_xlabel('X (Horizontal) [m]')
    ax.set_ylabel('Z (Depth) [m]')
    ax.set_zlabel('Y (Altitude) [m]')
    
    # Set dynamic limits based on the C generation
    ax.set_xlim([BOUNDS['x_min'], BOUNDS['x_max']])
    ax.set_ylim([BOUNDS['z_min'], BOUNDS['z_max']])
    ax.set_zlim([BOUNDS['y_min'], min(BOUNDS['y_max'], 50.0)])
    
    ax.legend(loc='upper right')
    
    def update(frame):
        pos = np.array([xs[frame], ys[frame], zs[frame]])
        drone_marker.set_data([pos[0]], [pos[2]])
        drone_marker.set_3d_properties([pos[1]])
        
        for i, ray_dir in enumerate(rays_dirs):
            hit_dist = compute_ray_hit(pos, ray_dir)
            end_pos = pos + ray_dir * hit_dist
            ray_lines[i].set_data([pos[0], end_pos[0]], [pos[2], end_pos[2]])
            ray_lines[i].set_3d_properties([pos[1], end_pos[1]])
            if hit_dist < MAX_LIDAR_RANGE:
                ray_lines[i].set_color('red')
                ray_lines[i].set_alpha(0.8)
            else:
                ray_lines[i].set_color('cyan')
                ray_lines[i].set_alpha(0.2)
                
        ax.set_title(f"Procedural Env (Seed in C) | T = {times[frame]:.2f}s")
        return [drone_marker] + ray_lines

    ani = FuncAnimation(fig, update, frames=range(0, len(xs), 4), interval=50, blit=False)
    plt.show()

if __name__ == '__main__':
    main()