#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// --- LIDAR SETTINGS ---
#define MAX_LIDAR_RANGE 20.0
#define NUM_RAYS 64
#define MAX_OBSTACLES 10 // Maximum memory allocation for mountains

#define N_DRONES 4
#define DRONE_RADIUS 1.0

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
extern double target_x[N_DRONES], target_y[N_DRONES], target_z[N_DRONES];
extern Obstacle3D obstacles[MAX_OBSTACLES];

// --- FUNCTIONS ---
void init_lidar(void);
void compute_lidar_rays(int drone_index, double drone_x, double drone_y, double drone_z, double current_x[N_DRONES], double current_y[N_DRONES], double current_z[N_DRONES], int active[N_DRONES], double distances[NUM_RAYS]);
void print_lidar_rays(double distances[NUM_RAYS], double current_time, double dx, double dy, double dz);

// NEW: Procedural Environment Generator
void generate_random_environment(unsigned int seed, double out_startX[N_DRONES], double out_startY[N_DRONES], double out_startZ[N_DRONES]);
// Exports the generated environment to a file for Python visualization
void export_environment(const char* filename);
#endif