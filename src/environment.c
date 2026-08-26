#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define the global map variables
double map_x_min = -50.0;
double map_x_max =  50.0;
double map_y_min =   0.0;
double map_y_max = 100.0;
double map_z_min = -50.0;
double map_z_max =  50.0;

int num_active_obstacles = 0;
Obstacle3D obstacles[MAX_OBSTACLES];
static LidarRay rays[NUM_RAYS];
double target_x[N_DRONES], target_y[N_DRONES], target_z[N_DRONES];
static unsigned int env_rand_state = 1;

static int env_rand(void) {
    env_rand_state = env_rand_state * 1103515245 + 12345;
    return (unsigned int)(env_rand_state / 65536) % 32768;
}

// Helper: Generates a random double between min and max
static double env_rand_double(double min, double max) {
    return min + ((double)env_rand() / 32767.0) * (max - min);
}

void generate_random_environment(unsigned int seed, double out_startX[N_DRONES], double out_startY[N_DRONES], double out_startZ[N_DRONES]) {
    // 1. Lock the Random Number Generator to the specific Generation Seed
    env_rand_state = seed;

    // 2. Randomize Patrol Area bounds (e.g., between 40m and 80m from center)
    map_x_max = env_rand_double(40.0, 80.0);
    map_x_min = -map_x_max;
    map_z_max = env_rand_double(40.0, 80.0);
    map_z_min = -map_z_max;
    map_y_min = 0.0; // Sea floor is always 0
    map_y_max = env_rand_double(40.0, 50.0); // Altitude ceiling

    // 3. Randomize Mountains (Amount, Size, Position)
    num_active_obstacles = 2 + (env_rand() % 5); // 2 to 6 mountains
    
    for (int i = 0; i < num_active_obstacles; i++) {
        obstacles[i].radius = env_rand_double(8.0, 25.0);
        // Keep mountains completely inside the map boundaries
        obstacles[i].x = env_rand_double(map_x_min + obstacles[i].radius, map_x_max - obstacles[i].radius);
        obstacles[i].z = env_rand_double(map_z_min + obstacles[i].radius, map_z_max - obstacles[i].radius);
        obstacles[i].y = 0.0; // Grounded on the sea floor
    }

    // 4. Find Safe Spawn Positions
    for (int d = 0; d < N_DRONES; d++) {
        int safe_spawn = 0;
        while (!safe_spawn) {
            out_startX[d] = env_rand_double(map_x_min + 10.0, map_x_max - 10.0);
            out_startZ[d] = env_rand_double(map_z_min + 10.0, map_z_max - 10.0);
            out_startY[d] = env_rand_double(10.0, map_y_max - 10.0);

            safe_spawn = 1;
            // Verify collision against all active mountains
            for (int i = 0; i < num_active_obstacles; i++) {
                double dx = out_startX[d] - obstacles[i].x;
                double dy = out_startY[d] - obstacles[i].y;
                double dz = out_startZ[d] - obstacles[i].z;
                
                double safe_distance = obstacles[i].radius + 5.0;
                if ((dx*dx + dy*dy + dz*dz) <= (safe_distance * safe_distance)) {
                    safe_spawn = 0; 
                    break;
                }
            }
            
            // Verify distance from other already spawned drones
            if (safe_spawn) {
                for (int other = 0; other < d; other++) {
                    double dx = out_startX[d] - out_startX[other];
                    double dy = out_startY[d] - out_startY[other];
                    double dz = out_startZ[d] - out_startZ[other];
                    double dist2 = dx*dx + dy*dy + dz*dz;
                    if (dist2 < (4.0 * DRONE_RADIUS * DRONE_RADIUS + 10.0)) { // Give some buffer room
                        safe_spawn = 0;
                        break;
                    }
                }
            }
        }
    }

    // 5. Find Safe Target Positions
    for (int d = 0; d < N_DRONES; d++) {
        int safe_target = 0;
        while (!safe_target) {
            target_x[d] = env_rand_double(map_x_min + 10.0, map_x_max - 10.0);
            target_z[d] = env_rand_double(map_z_min + 10.0, map_z_max - 10.0);
            target_y[d] = env_rand_double(10.0, 15.0);

            safe_target = 1;
            for (int i = 0; i < num_active_obstacles; i++) {
                double dx = target_x[d] - obstacles[i].x;
                double dy = target_y[d] - obstacles[i].y;
                double dz = target_z[d] - obstacles[i].z;
                double safe_distance = obstacles[i].radius + 5.0;
                if ((dx*dx + dy*dy + dz*dz) <= (safe_distance * safe_distance)) {
                    safe_target = 0; 
                    break;
                }
            }
        }
    }
}

void init_lidar(void) {
    // Distribute rays uniformly in 3D using Fibonacci Sphere
    double phi = M_PI * (3.0 - sqrt(5.0)); // Golden angle

    for (int i = 0; i < NUM_RAYS; i++) {
        // Y goes from 1 to -1 (Altitude axis)
        double y = 1.0 - (i / (double)(NUM_RAYS - 1)) * 2.0; 
        double radius = sqrt(1.0 - y * y);
        
        double theta = phi * i;
        
        double x = cos(theta) * radius;
        double z = sin(theta) * radius;

        // Store the normalized direction vector
        rays[i].dir_x = x;
        rays[i].dir_y = y;
        rays[i].dir_z = z;
    }
}

// Computes the exact distance to the closest hit for a single ray
static double shoot_single_ray(int drone_index, double ox, double oy, double oz, double dx, double dy, double dz, double current_x[N_DRONES], double current_y[N_DRONES], double current_z[N_DRONES], int active[N_DRONES]) {
    double min_dist = MAX_LIDAR_RANGE;

    // 1. Check Intersection with 3D Obstacles
    for (int i = 0; i < num_active_obstacles; i++) {
        double lx = obstacles[i].x - ox;
        double ly = obstacles[i].y - oy;
        double lz = obstacles[i].z - oz;

        // --- NUOVO CONTROLLO: COLLISIONE INTERNA ---
        // Squared distance from drone to obstacle center
        double L2 = (lx*lx + ly*ly + lz*lz);
        double radius2 = obstacles[i].radius * obstacles[i].radius;

        // If drone is completely inside the obstacle, distance is 0
        if (L2 <= radius2) {
            return 0.0; // Collision! Immediate return.
        }
        // -------------------------------------------

        double tca = lx * dx + ly * dy + lz * dz;

        // If tca < 0, the obstacle is strictly behind the ray
        if (tca < 0) continue;

        double d2 = L2 - (tca*tca);

        // If d2 > radius squared, the ray misses the sphere
        if (d2 > radius2) continue;

        double thc = sqrt(radius2 - d2);
        double hit_dist = tca - thc;

        if (hit_dist > 0.0 && hit_dist < min_dist) {
            min_dist = hit_dist;
        }
    }

    // 1.5 Check Intersection with Other Drones
    for (int i = 0; i < N_DRONES; i++) {
        if (i == drone_index || !active[i]) continue;
        
        double lx = current_x[i] - ox;
        double ly = current_y[i] - oy;
        double lz = current_z[i] - oz;

        double L2 = (lx*lx + ly*ly + lz*lz);
        double radius2 = DRONE_RADIUS * DRONE_RADIUS;

        if (L2 <= radius2) {
            return 0.0; 
        }

        double tca = lx * dx + ly * dy + lz * dz;
        if (tca < 0) continue;

        double d2 = L2 - (tca*tca);
        if (d2 > radius2) continue;

        double thc = sqrt(radius2 - d2);
        double hit_dist = tca - thc;

        if (hit_dist > 0.0 && hit_dist < min_dist) {
            min_dist = hit_dist;
        }
    }

    // 2. Check Intersection with Map Boundaries (Ray-Plane Intersection)
    double t_bounds;
    
    // Check X bounds
    if (dx > 0) { t_bounds = (map_x_max - ox) / dx; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    if (dx < 0) { t_bounds = (map_x_min - ox) / dx; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    
    // Check Y bounds (Altitude)
    if (dy > 0) { t_bounds = (map_y_max - oy) / dy; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    if (dy < 0) { t_bounds = (map_y_min - oy) / dy; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    
    // Check Z bounds (Depth)
    if (dz > 0) { t_bounds = (map_z_max - oz) / dz; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    if (dz < 0) { t_bounds = (map_z_min - oz) / dz; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }

    return min_dist;
}

void compute_lidar_rays(int drone_index, double drone_x, double drone_y, double drone_z, double current_x[N_DRONES], double current_y[N_DRONES], double current_z[N_DRONES], int active[N_DRONES], double distances[NUM_RAYS]) {
    for (int i = 0; i < NUM_RAYS; i++) {
        distances[i] = shoot_single_ray(drone_index, drone_x, drone_y, drone_z, rays[i].dir_x, rays[i].dir_y, rays[i].dir_z, current_x, current_y, current_z, active);
    }
}

void print_lidar_rays(double distances[NUM_RAYS], double current_time, double dx, double dy, double dz) {
    printf("\n--- LIDAR %d RAYS | T=%.1fs | Pos: (%.1f, %.1f, %.1f) ---\n", NUM_RAYS, current_time, dx, dy, dz);
    
    int hits = 0;
    for (int i = 0; i < NUM_RAYS; i++) {
        if (distances[i] < MAX_LIDAR_RANGE) {
            hits++;
        }
    }
    printf("Status: %d rays hit an object/boundary out of %d.\n", hits, NUM_RAYS);
    
    // Print the first 5 rays just as a sample to avoid terminal spam
    for (int i = 0; i < 5; i++) {
        printf("Ray %02d (dir: %5.2f, %5.2f, %5.2f) -> Distance: %5.2f m\n", 
               i, rays[i].dir_x, rays[i].dir_y, rays[i].dir_z, distances[i]);
    }
    printf("----------------------------------------------------------\n");
}
void export_environment(const char* filename) {
    FILE* f = fopen(filename, "w");
    if (f == NULL) {
        printf("Warning: Could not open %s for writing.\n", filename);
        return;
    }
    
    // Write Map Boundaries
    fprintf(f, "BOUNDS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
            map_x_min, map_x_max, map_y_min, map_y_max, map_z_min, map_z_max);
    
    // Write Active Obstacles
    for (int i = 0; i < num_active_obstacles; i++) {
        fprintf(f, "OBS,%.2f,%.2f,%.2f,%.2f\n", 
                obstacles[i].x, obstacles[i].y, obstacles[i].z, obstacles[i].radius);
    }

    // Write all targets
    for (int d = 0; d < N_DRONES; d++) {
        fprintf(f, "TARGET,%.2f,%.2f,%.2f\n", target_x[d], target_y[d], target_z[d]);
    }
    
    fclose(f);
}