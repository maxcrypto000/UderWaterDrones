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

# Will be populated dynamically from C output
BOUNDS = {}
OBSTACLES = []
TARGETS = []
N_DRONES = 4

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
                elif parts[0] == 'TARGET':
                    TARGETS.append({'x': float(parts[1]), 'y': float(parts[2]), 'z': float(parts[3])})
    except FileNotFoundError:
        print(f"Error: {filename} not found. Did you run the C simulation first?")
        exit(1)

def load_telemetry(filename):
    times = []
    drones_data = [{'x': [], 'y': [], 'z': []} for _ in range(N_DRONES)]
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row['time']))
                for d in range(N_DRONES):
                    # Handle potential NaN strings if drones crash
                    try:
                        drones_data[d]['x'].append(float(row[f'x{d}']))
                        drones_data[d]['y'].append(float(row[f'y{d}']))
                        drones_data[d]['z'].append(float(row[f'z{d}']))
                    except ValueError:
                        # If it's NaN, just repeat the last known position
                        if len(drones_data[d]['x']) > 0:
                            drones_data[d]['x'].append(drones_data[d]['x'][-1])
                            drones_data[d]['y'].append(drones_data[d]['y'][-1])
                            drones_data[d]['z'].append(drones_data[d]['z'][-1])
                        else:
                            drones_data[d]['x'].append(0.0)
                            drones_data[d]['y'].append(0.0)
                            drones_data[d]['z'].append(0.0)
    except FileNotFoundError:
        print(f"Error: {filename} not found.")
        exit(1)
    return times, drones_data

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

def draw_hemisphere(ax, x_center, y_center, z_center, radius, color='darkorange', alpha=0.5):
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi / 2, 15) 
    x = x_center + radius * np.outer(np.cos(u), np.sin(v))
    z = z_center + radius * np.outer(np.sin(u), np.sin(v))
    y = y_center + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, z, y, color=color, alpha=alpha, edgecolor='k', linewidth=0.2)

def draw_patrol_area(ax):
    """Draws the bounding box representing the patrol area"""
    x_min, x_max = BOUNDS['x_min'], BOUNDS['x_max']
    y_min, y_max = BOUNDS['y_min'], BOUNDS['y_max']
    z_min, z_max = BOUNDS['z_min'], BOUNDS['z_max']

    # Vertices of the patrol volume (mapping Y to Z-axis for matplotlib)
    # Floor is already drawn in main(), so we draw walls and ceiling
    alpha_walls = 0.05
    wall_color = 'gray'

    # 1. Ceiling
    xx, zz = np.meshgrid([x_min, x_max], [z_min, z_max])
    yy = np.full_like(xx, y_max)
    ax.plot_surface(xx, zz, yy, color=wall_color, alpha=alpha_walls)

    # 2. Back Wall (Z min)
    xx, yy = np.meshgrid([x_min, x_max], [y_min, y_max])
    zz = np.full_like(xx, z_min)
    ax.plot_surface(xx, zz, yy, color=wall_color, alpha=alpha_walls)

    # 3. Front Wall (Z max)
    zz = np.full_like(xx, z_max)
    ax.plot_surface(xx, zz, yy, color=wall_color, alpha=alpha_walls)

    # 4. Left Wall (X min)
    zz, yy = np.meshgrid([z_min, z_max], [y_min, y_max])
    xx = np.full_like(zz, x_min)
    ax.plot_surface(xx, zz, yy, color=wall_color, alpha=alpha_walls)

    # 5. Right Wall (X max)
    xx = np.full_like(zz, x_max)
    ax.plot_surface(xx, zz, yy, color=wall_color, alpha=alpha_walls)

    # Draw wireframe edges to make the bounds clearer
    edges = [
        ([x_min, x_max], [z_min, z_min], [y_max, y_max]),
        ([x_min, x_max], [z_max, z_max], [y_max, y_max]),
        ([x_min, x_min], [z_min, z_max], [y_max, y_max]),
        ([x_max, x_max], [z_min, z_max], [y_max, y_max]),
        ([x_min, x_min], [z_min, z_min], [y_min, y_max]),
        ([x_max, x_max], [z_min, z_min], [y_min, y_max]),
        ([x_min, x_min], [z_max, z_max], [y_min, y_max]),
        ([x_max, x_max], [z_max, z_max], [y_min, y_max])
    ]
    for ex, ez, ey in edges:
        ax.plot(ex, ez, ey, color='black', alpha=0.3, linewidth=1, linestyle='--')


def compute_ray_hit(drone_idx, origin, direction, all_drone_positions):
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
            
    # Check Other Drones
    drone_radius = 1.0
    for d, d_pos in enumerate(all_drone_positions):
        if d == drone_idx: continue
        L_vec = d_pos - origin
        if np.dot(L_vec, L_vec) <= drone_radius * drone_radius: return 0.0
        tca = np.dot(L_vec, direction)
        if tca < 0: continue
        d2 = np.dot(L_vec, L_vec) - tca * tca
        if d2 > drone_radius * drone_radius: continue
        thc = math.sqrt(drone_radius * drone_radius - d2)
        hit_dist = tca - thc
        if 0 < hit_dist < min_dist:
            min_dist = hit_dist

    # Check Sea Floor (Y = 0)
    if direction[1] < 0:
        t_floor = (BOUNDS['y_min'] - origin[1]) / direction[1]
        if 0 < t_floor < min_dist: min_dist = t_floor
        
    # Check Ceiling (Y max)
    if direction[1] > 0:
        t_ceil = (BOUNDS['y_max'] - origin[1]) / direction[1]
        if 0 < t_ceil < min_dist: min_dist = t_ceil
        
    # Check X boundaries
    if direction[0] > 0:
        t_x = (BOUNDS['x_max'] - origin[0]) / direction[0]
        if 0 < t_x < min_dist: min_dist = t_x
    elif direction[0] < 0:
        t_x = (BOUNDS['x_min'] - origin[0]) / direction[0]
        if 0 < t_x < min_dist: min_dist = t_x
        
    # Check Z boundaries
    if direction[2] > 0:
        t_z = (BOUNDS['z_max'] - origin[2]) / direction[2]
        if 0 < t_z < min_dist: min_dist = t_z
    elif direction[2] < 0:
        t_z = (BOUNDS['z_min'] - origin[2]) / direction[2]
        if 0 < t_z < min_dist: min_dist = t_z

    return min_dist

def main():
    load_environment(ENVIRONMENT_FILE)
    times, drones_data = load_telemetry(TELEMETRY_FILE)
    rays_dirs = get_fibonacci_sphere(NUM_RAYS)
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = ['blue', 'green', 'magenta', 'cyan']
    
    # Draw Trajectories and Starts
    for d in range(N_DRONES):
        c = colors[d % len(colors)]
        xs = drones_data[d]['x']
        ys = drones_data[d]['y']
        zs = drones_data[d]['z']
        if len(xs) > 0:
            ax.plot(xs, zs, ys, color=c, linewidth=2, alpha=0.8, label=f'Trajectory {d}')
            ax.scatter(xs[0], zs[0], ys[0], color='lime', s=100, marker='*', edgecolor='black', zorder=5)
    
    # Draw Target Positions (Gold Star)
    for d, tg in enumerate(TARGETS):
        ax.scatter(tg['x'], tg['z'], tg['y'], color='gold', s=150, marker='*', edgecolor='black', zorder=5, label=f'Target {d}')
        
    # Draw Sea Floor
    xx, zz = np.meshgrid([BOUNDS['x_min'], BOUNDS['x_max']], [BOUNDS['z_min'], BOUNDS['z_max']])
    yy = np.full_like(xx, BOUNDS['y_min'])
    ax.plot_surface(xx, zz, yy, color='blue', alpha=0.15)

    # Draw Patrol Area Walls/Ceiling
    draw_patrol_area(ax)
    
    # Draw Dynamic Obstacles
    for obs in OBSTACLES:
        draw_hemisphere(ax, obs[0], obs[1], obs[2], obs[3])

    drone_markers = []
    ray_lines = []
    for d in range(N_DRONES):
        c = colors[d % len(colors)]
        marker, = ax.plot([], [], [], 'o', color=c, markersize=8, label=f'Drone {d}')
        drone_markers.append(marker)
        
        # Only draw lidars for drone 0 to avoid massive lag and clutter, 
        # or we could do it for all but alpha=0.1
        drone_rays = [ax.plot([], [], [], color=c, alpha=0.1, linewidth=1)[0] for _ in range(NUM_RAYS)]
        ray_lines.append(drone_rays)

    ax.set_xlabel('X (Horizontal) [m]')
    ax.set_ylabel('Z (Depth) [m]')
    ax.set_zlabel('Y (Altitude) [m]')
    
    # Expand limits slightly beyond the bounds for better viewing
    margin = 10
    ax.set_xlim([BOUNDS['x_min'] - margin, BOUNDS['x_max'] + margin])
    ax.set_ylim([BOUNDS['z_min'] - margin, BOUNDS['z_max'] + margin])
    ax.set_zlim([BOUNDS['y_min'], BOUNDS['y_max'] + margin])
    
    ax.legend(loc='upper right', fontsize='small')
    
    def update(frame):
        all_drones_pos = []
        for d in range(N_DRONES):
            xs = drones_data[d]['x']
            ys = drones_data[d]['y']
            zs = drones_data[d]['z']
            all_drones_pos.append(np.array([xs[frame], ys[frame], zs[frame]]))
            
        artists = []
        
        for d in range(N_DRONES):
            pos = all_drones_pos[d]
            drone_markers[d].set_data([pos[0]], [pos[2]])
            drone_markers[d].set_3d_properties([pos[1]])
            artists.append(drone_markers[d])
            
            # Update rays for this drone
            for i, ray_dir in enumerate(rays_dirs):
                hit_dist = compute_ray_hit(d, pos, ray_dir, all_drones_pos)
                end_pos = pos + ray_dir * hit_dist
                ray_lines[d][i].set_data([pos[0], end_pos[0]], [pos[2], end_pos[2]])
                ray_lines[d][i].set_3d_properties([pos[1], end_pos[1]])
                if hit_dist < MAX_LIDAR_RANGE:
                    ray_lines[d][i].set_color('red')
                    ray_lines[d][i].set_alpha(0.8)
                else:
                    ray_lines[d][i].set_color(colors[d % len(colors)])
                    ray_lines[d][i].set_alpha(0.1)
                artists.append(ray_lines[d][i])
                
        ax.set_title(f"Simulation MARL | T = {times[frame]:.2f}s | {N_DRONES} Drones")
        return artists

    num_frames = len(times)
    ani = FuncAnimation(fig, update, frames=range(0, num_frames, 4), interval=50, blit=False)
    plt.show()

if __name__ == '__main__':
    main()