#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// --- LIDAR SETTINGS ---
#define MAX_LIDAR_RANGE 20.0
#define NUM_RAYS 64
#define MAX_OBSTACLES 10 // Maximum memory allocation for mountains

typedef struct {
    double x;
    double y;
    double z;
    double radius;
} Obstacle3D;

typedef struct {
    double dir_x;
    double dir_y;
    double dir_z;
} LidarRay;

// --- GLOBAL MAP VARIABLES ---
extern double map_x_min, map_x_max;
extern double map_y_min, map_y_max;
extern double map_z_min, map_z_max;
extern int num_active_obstacles;
extern Obstacle3D obstacles[MAX_OBSTACLES];

// --- FUNCTIONS ---
void init_lidar(void);
void compute_lidar_rays(double drone_x, double drone_y, double drone_z, double distances[NUM_RAYS]);
void print_lidar_rays(double distances[NUM_RAYS], double current_time, double dx, double dy, double dz);

// NEW: Procedural Environment Generator
void generate_random_environment(unsigned int seed, double* out_startX, double* out_startY, double* out_startZ);
// Exports the generated environment to a file for Python visualization
void export_environment(const char* filename);
#endif