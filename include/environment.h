#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// --- LIDAR SETTINGS ---
#define MAX_LIDAR_RANGE 20.0
#define NUM_RAYS 64
#define MAX_OBSTACLES 10 // Maximum memory allocation for mountains

#define N_DRONES 5
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
extern int num_active_drones;
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
// --- DYNAMIC MISSION ---

/**
 * @brief Generates a random environment specifically for the dynamic mission.
 * 
 * Unlike standard training where drones spawn in mid-air, this variant ensures
 * drones spawn safely resting on the seabed (Y = DRONE_RADIUS) while avoiding 
 * initial collisions with randomly generated mountainous obstacles.
 *
 * @param seed Random seed for reproducible generation.
 * @param req_drones Desired number of active drones (clamps to N_DRONES).
 * @param req_obstacles Desired number of static mountains (clamps to MAX_OBSTACLES).
 * @param out_startX Array to store the generated X starting coordinates.
 * @param out_startY Array to store the generated Y starting coordinates.
 * @param out_startZ Array to store the generated Z starting coordinates.
 */
void generate_mission_environment(unsigned int seed, int req_drones, int req_obstacles, double out_startX[N_DRONES], double out_startY[N_DRONES], double out_startZ[N_DRONES]);

/**
 * @brief Generates a localized, obstacle-free target waypoint for a drone.
 *
 * Computes a random (x, y, z) target within a specified radial neighborhood 
 * of the drone's current position. The generated target is guaranteed to 
 * respect map boundaries and not intersect any obstacle geometries.
 *
 * @param cx Current drone X coordinate.
 * @param cy Current drone Y coordinate.
 * @param cz Current drone Z coordinate.
 * @param tx Pointer to output target X coordinate.
 * @param ty Pointer to output target Y coordinate.
 * @param tz Pointer to output target Z coordinate.
 * @param radius The maximum radial distance for the new target.
 */
void generate_local_target(double cx, double cy, double cz, double* tx, double* ty, double* tz, double radius);
