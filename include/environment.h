/**
 * @file environment.h
 * @brief Environment generation and LIDAR sensor simulation for the drone swarm.
 *
 * This module is responsible for defining the physical 3D world in which the drones 
 * operate. It handles procedural generation of boundaries, static obstacles (mountains),
 * and random target placement. Furthermore, it implements the math for a volumetric 
 * LIDAR sensor via raycasting to detect obstacles and other drones.
 */

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// --- LIDAR SETTINGS ---

/** @brief Maximum range of the LIDAR sensor in meters. */
#define MAX_LIDAR_RANGE 20.0

/** @brief Total number of rays cast per drone per step (Fibonacci Sphere). */
#define NUM_RAYS 64

/** @brief Maximum number of static obstacles (mountains) allowed in the environment. */
#define MAX_OBSTACLES 10 

/** @brief Maximum number of active drones in the swarm. */
#define N_DRONES 5

/** @brief Physical radius of each drone, used for drone-to-drone collision detection. */
#define DRONE_RADIUS 1.0

/**
 * @struct Obstacle3D
 * @brief Defines a static spherical (or hemispherical) obstacle.
 */
typedef struct {
    double x;      /**< X coordinate of the obstacle center. */
    double y;      /**< Y coordinate of the obstacle center. */
    double z;      /**< Z coordinate of the obstacle center. */
    double radius; /**< Radius of the obstacle. */
} Obstacle3D;

/**
 * @struct LidarRay
 * @brief Defines a normalized directional vector for a single LIDAR ray.
 */
typedef struct {
    double dir_x; /**< Normalized X direction. */
    double dir_y; /**< Normalized Y direction. */
    double dir_z; /**< Normalized Z direction. */
} LidarRay;

// --- GLOBAL MAP VARIABLES ---

extern double map_x_min, map_x_max;
extern double map_y_min, map_y_max;
extern double map_z_min, map_z_max;

extern int num_active_obstacles;
extern int num_active_drones;

/** @brief Global array containing the current dynamic target for each drone. */
extern double target_x[N_DRONES], target_y[N_DRONES], target_z[N_DRONES];

/** @brief Global array storing the currently spawned static obstacles. */
extern Obstacle3D obstacles[MAX_OBSTACLES];

// --- FUNCTIONS ---

/**
 * @brief Initializes the LIDAR ray directions using a Fibonacci Sphere algorithm.
 * 
 * Generates an almost perfectly uniform distribution of points across a sphere,
 * which serves as the directional vectors for the LIDAR sensor.
 */
void init_lidar(void);

/**
 * @brief Computes all LIDAR ray distances for a specific drone.
 *
 * Casts NUM_RAYS against map bounds, static obstacles, and other drones.
 *
 * @param drone_index The index of the drone casting the rays.
 * @param drone_x X coordinate of the drone.
 * @param drone_y Y coordinate of the drone.
 * @param drone_z Z coordinate of the drone.
 * @param current_x Array of all drone X coordinates.
 * @param current_y Array of all drone Y coordinates.
 * @param current_z Array of all drone Z coordinates.
 * @param active Array indicating which drones are active.
 * @param distances Output array to store the intersection distances.
 */
void compute_lidar_rays(int drone_index, double drone_x, double drone_y, double drone_z, double current_x[N_DRONES], double current_y[N_DRONES], double current_z[N_DRONES], int active[N_DRONES], double distances[NUM_RAYS]);

/**
 * @brief Prints the status of the LIDAR rays to the console (for debugging).
 */
void print_lidar_rays(double distances[NUM_RAYS], double current_time, double dx, double dy, double dz);

/**
 * @brief Procedurally generates a random training environment.
 *
 * Creates random bounds, places random mountains, and spawns drones safely
 * in mid-air ensuring no initial collisions occur. Used during ES training.
 *
 * @param seed Random seed for reproducible generation.
 * @param out_startX Output array for drone X spawn coordinates.
 * @param out_startY Output array for drone Y spawn coordinates.
 * @param out_startZ Output array for drone Z spawn coordinates.
 */
void generate_random_environment(unsigned int seed, double out_startX[N_DRONES], double out_startY[N_DRONES], double out_startZ[N_DRONES]);

/**
 * @brief Exports the generated map and obstacles to a CSV file.
 *
 * @param filename Destination CSV file path.
 */
void export_environment(const char* filename);

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

#endif
