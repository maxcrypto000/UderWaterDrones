#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// --- MAP LIMITS ---
#define MAP_X_MIN -500.0
#define MAP_X_MAX  500.0
#define MAP_Y_MIN   0.0  // Altitude
#define MAP_Y_MAX 100.0  
#define MAP_Z_MIN -500.0
#define MAP_Z_MAX  500.0

// --- LIDAR SETTINGS ---
#define MAX_LIDAR_RANGE 20.0
#define NUM_RAYS 64 // 64 rays distributed uniformly in 3D space

// --- OBSTACLES (3D Spheres) ---
typedef struct {
    double x;
    double y; // Altitude
    double z; // Depth
    double radius;
} Obstacle3D;

// --- RAYCASTING STRUCTURES ---
typedef struct {
    double dir_x;
    double dir_y;
    double dir_z;
} LidarRay;

// --- FUNCTIONS ---
// Initializes the ray directions using a Fibonacci Sphere distribution
void init_lidar(void);

// Computes the intersection of all rays with boundaries and obstacles
void compute_lidar_rays(double drone_x, double drone_y, double drone_z, double distances[NUM_RAYS]);

// Helper to print a summary of the Lidar readings
void print_lidar_rays(double distances[NUM_RAYS], double current_time, double dx, double dy, double dz);

#endif