#include "environment.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#define NUM_OBSTACLES 1
static Obstacle3D obstacles[NUM_OBSTACLES] = {
    // Center at Altitude Y = 0.0. Radius = 15.0m
    {15.0, 0.0, 15.0, 12.0}   
};

// Global array storing the normalized directions of our 64 rays
static LidarRay rays[NUM_RAYS];

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
static double shoot_single_ray(double ox, double oy, double oz, double dx, double dy, double dz) {
    double min_dist = MAX_LIDAR_RANGE;

    // 1. Check Intersection with 3D Obstacles
    for (int i = 0; i < NUM_OBSTACLES; i++) {
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

    // 2. Check Intersection with Map Boundaries (Ray-Plane Intersection)
    double t_bounds;
    
    // Check X bounds
    if (dx > 0) { t_bounds = (MAP_X_MAX - ox) / dx; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    if (dx < 0) { t_bounds = (MAP_X_MIN - ox) / dx; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    
    // Check Y bounds (Altitude)
    if (dy > 0) { t_bounds = (MAP_Y_MAX - oy) / dy; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    if (dy < 0) { t_bounds = (MAP_Y_MIN - oy) / dy; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    
    // Check Z bounds (Depth)
    if (dz > 0) { t_bounds = (MAP_Z_MAX - oz) / dz; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }
    if (dz < 0) { t_bounds = (MAP_Z_MIN - oz) / dz; if (t_bounds > 0 && t_bounds < min_dist) min_dist = t_bounds; }

    return min_dist;
}

void compute_lidar_rays(double drone_x, double drone_y, double drone_z, double distances[NUM_RAYS]) {
    for (int i = 0; i < NUM_RAYS; i++) {
        distances[i] = shoot_single_ray(drone_x, drone_y, drone_z, rays[i].dir_x, rays[i].dir_y, rays[i].dir_z);
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