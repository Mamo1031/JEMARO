#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <math.h>
#include <errno.h>
#include <sys/select.h>
#include <stdbool.h>
#include <pthread.h>
#include "utils.h"


// Global file pointers for log files
FILE *debug, *errors;

// Global variables for inter-process communication and physics parameters
pid_t wd_pid;  // Process ID of the watchdog

// Force-related parameters (rho0, rho1, and eta are scaling factors for the potential field forces)
float rho0 = 2.0f, rho1 = 0.5f, eta = 40.0f;

Game game;           // Global game configuration
Drone *drone;        // Pointer to the shared drone structure
Score *score;        // Pointer to the shared score
int numObstacles, numTargets;  // Number of obstacles and targets

Object *obstacles, *targets;   // Arrays holding obstacle and target objects

// Coefficient for attractive force applied to targets
float attractionCoefficient = 5.0f;

//---------------------------------------------------------------------
// Calculate the friction force for a given velocity.
// Returns a force opposing the motion.
float calculate_friction_force(float velocity) {
    return -FRICTION_COEFFICIENT * velocity;
}

//---------------------------------------------------------------------
// Calculate the attractive force in the X direction between two positions.
// Only applied if the absolute difference is less than rho1.
float calculate_attractive_force_x(int pos, int targetPos) {
    if (abs(pos - targetPos) < rho1)
        return -attractionCoefficient * (pos - targetPos);
    else
        return 0.0f;
}    

// Calculate the attractive force in the Y direction.
float calculate_attractive_force_y(int pos, int targetPos) {
    if (abs(pos - targetPos) < rho1)
        return -attractionCoefficient * (pos - targetPos);
    else
        return 0.0f;
}

//---------------------------------------------------------------------
// Calculate the repulsive force in the X direction exerted by an object
// located at (xo, yo) on the drone. The force is scaled based on the distance.
float calculate_repulsive_force_x(Drone droneData, int xo, int yo) {
    // Compute Euclidean distance between the drone and the object.
    float distance = sqrt(pow(droneData.pos_x - xo, 2) + pow(droneData.pos_y - yo, 2));
    if (distance < 0.5f) 
        distance = 0.5f;  // Prevent division by zero or too high force
    float angle = atan2(droneData.pos_y - yo, droneData.pos_x - xo);
    float forceX;

    if (distance < rho0) {
        forceX = eta * (1.0f / distance - 1.0f / rho0) * cos(angle) * fabs(droneData.vel_x);
    } else {
        forceX = 0.0f;
    }

    // Clamp the force to the maximum allowed repulsive force.
    if (forceX > MAX_FREP) forceX = MAX_FREP;
    if (forceX < -MAX_FREP) forceX = -MAX_FREP;

    return forceX;
}

// Calculate the repulsive force in the Y direction.
float calculate_repulsive_force_y(Drone droneData, int xo, int yo) {
    float distance = sqrt(pow(droneData.pos_x - xo, 2) + pow(droneData.pos_y - yo, 2));
    if (distance < 0.5f) 
        distance = 0.5f;
    float angle = atan2(droneData.pos_y - yo, droneData.pos_x - xo);
    float forceY;

    if (distance < rho0) {
        forceY = eta * (1.0f / distance - 1.0f / rho0) * sin(angle) * fabs(droneData.vel_y);
    } else {
        forceY = 0.0f;
    }

    if (forceY > MAX_FREP) forceY = MAX_FREP;
    if (forceY < -MAX_FREP) forceY = -MAX_FREP;

    return forceY;
}

//---------------------------------------------------------------------
// Check if the drone has "hit" any object (obstacle or target) within a threshold.
// Depending on the object type, different forces are applied.
// Parameters:
//   drone: pointer to the Drone structure.
//   objects: array of objects (either obstacles or targets).
//   objectCount: number of objects in the array.
//   forces: output array where forces[0] is force in X and forces[1] is force in Y.
void check_hit(Drone *drone, Object *objects, int objectCount, float *forces) {
    forces[0] = 0.0f; // Force in X direction
    forces[1] = 0.0f; // Force in Y direction
    float index_target;
    for (int i = 0; i < objectCount; i++) {
        /* 
         * We add 0.5 to the object's position so that the distance is measured
         * from the center of the object rather than its top-left corner.
         */
        float distance = sqrt(pow(drone->pos_x - (objects[i].pos_x + 0.5f), 2) +
                              pow(drone->pos_y - (objects[i].pos_y + 0.5f), 2));
        // Check if the object is within the hit threshold and not already "hit" (unless it is an obstacle).
        if (distance <= HIT_THR && (!objects[i].hit || objects[i].type == 'o')) {
            if (objects[i].type == 'o') {
                // For obstacles, apply a repulsive force.
                score->score -= score->last_target/2; // the score decreases by half of the index-number of the last target reached
                score->last_target = 2; // after a collision with an obstacle, successive collisions cost 1 point of score only
                forces[0] += calculate_repulsive_force_x(*drone, objects[i].pos_x, objects[i].pos_y);
                forces[1] += calculate_repulsive_force_y(*drone, objects[i].pos_x, objects[i].pos_y);
            } else {
                // For targets, increase the score, mark the target as hit, and apply an attractive force.
                index_target = (float)objects[i].number;
                
                if (index_target == score->last_target + 1){
                    score->score += index_target; // if the targets are reached in order, the targets index-numbers are added
                } else {
                    score->score = 1.0f; // if the index of the target reached does not follow the index of the previous target reached, score +=1
                }
                score->last_target = index_target;
                objects[i].hit = true;
                forces[0] += calculate_attractive_force_x(drone->pos_x, objects[i].pos_x);
                forces[1] += calculate_attractive_force_y(drone->pos_y, objects[i].pos_y);  
            }
        }
    }
}

void check_geofences(Drone *drone, float *forces) {
    forces[0] = 0.0f; // Force in X direction
    forces[1] = 0.0f; // Force in Y direction

    // Check if the drone is within the hit threshold of the geofences
    float distance_up = fabs(drone->pos_y);
    float distance_down = fabs(drone->pos_y - game.max_y);
    float distance_left = fabs(drone->pos_x);
    float distance_right = fabs(drone->pos_x - game.max_x);

    if (distance_up <= HIT_THR){
        forces[1] += calculate_repulsive_force_x(*drone, drone->pos_x, 0);
    }
    if (distance_down <= HIT_THR){
        forces[1] += calculate_repulsive_force_x(*drone, drone->pos_x, game.max_y);
    }
    if (distance_left <= HIT_THR){
        forces[0] += calculate_repulsive_force_x(*drone, 0, drone->pos_y);
    }
    if (distance_right <= HIT_THR){
        forces[0] += calculate_repulsive_force_x(*drone, game.max_x, drone->pos_y);
    }
}

//---------------------------------------------------------------------
// Update the drone's position based on forces, friction, and collisions.
// dt: time step for the update.
void update_drone_position(Drone *drone, float dt) {
    float forceObsX = 0.0f, forceObsY = 0.0f;
    float forceTargetsX = 0.0f, forceTargetsY = 0.0f;
    float forceGeofenceX = 0.0f, forceGeofenceY = 0.0f;
    float forces[2];

    // Calculate friction forces for both directions.
    float frictionForceX = calculate_friction_force(drone->vel_x);
    float frictionForceY = calculate_friction_force(drone->vel_y);

    // Check collisions with obstacles.
    check_hit(drone, obstacles, numObstacles, forces);
    forceObsX = forces[0];
    forceObsY = forces[1];
    
    // Check collisions with targets.
    check_hit(drone, targets, numTargets, forces);
    forceTargetsX = forces[0];
    forceTargetsY = forces[1];

    // Check collisions with geofence.
    check_geofences(drone, forces);
    forceGeofenceX = forces[0];
    forceGeofenceY = forces[1];

    // Compute acceleration using Newton's second law: a = (F_total) / mass.
    float accelerationX = (drone->force_x + frictionForceX + forceObsX + forceTargetsX + forceGeofenceX) / MASS;
    float accelerationY = (drone->force_y + frictionForceY + forceObsY + forceTargetsY + forceGeofenceY) / MASS;

    // Update velocity using acceleration.
    drone->vel_x += accelerationX * dt;
    drone->vel_y += accelerationY * dt;
    // Update position using the equations of motion.
    drone->pos_x += drone->vel_x * dt + 0.5f * accelerationX * dt * dt;
    drone->pos_y += drone->vel_y * dt + 0.5f * accelerationY * dt * dt;

    // Boundary checking: ensure the drone remains within the game area.
    if (drone->pos_x < 0) { 
        drone->pos_x = 0; 
        drone->vel_x = 0; 
        drone->force_x = 0;
    }
    if (drone->pos_x >= game.max_x) { 
        drone->pos_x = game.max_x - 1; 
        drone->vel_x = 0; 
        drone->force_x = 0;
    }
    if (drone->pos_y < 0) {
        drone->pos_y = 0; 
        drone->vel_y = 0; 
        drone->force_y = 0;
    }
    if (drone->pos_y >= game.max_y) { 
        drone->pos_y = game.max_y - 1; 
        drone->vel_y = 0; 
        drone->force_y = 0;
    }
}

//---------------------------------------------------------------------
// Thread function that continuously updates the drone's position.
// It sleeps for 50 ms between updates.
void *update_drone_position_thread(void *arg) {
    (void)arg;  // Unused parameter
    while (1) {
        update_drone_position(drone, T);
        usleep(50000);  // Sleep for 50 milliseconds
    }
    return NULL;
}

//---------------------------------------------------------------------
// Process a key press and update the drone's force accordingly.
// The keys correspond to different movement directions.
// Note: Some cases subtract or add zero; these are kept for clarity.
void handle_key_pressed(char key, Drone *drone) {
    // Normalize the force when the displacement is diagonal, so that the norm of the input force is always equal to FORCE_MODULE.
    double NORM_DIAGONAL = sqrt(5)/2;
    switch (key) {
        case 'e': case 'E':
            drone->force_x -= FORCE_MODULE / NORM_DIAGONAL;
            drone->force_y -= FORCE_MODULE / (2*NORM_DIAGONAL);
            break;
        case 'r': case 'R':
            // No change to force_x; decrease force_y.
            drone->force_y -= FORCE_MODULE / 2;
            break;
        case 't': case 'T':
            drone->force_x += FORCE_MODULE / NORM_DIAGONAL;
            drone->force_y -= FORCE_MODULE / (2*NORM_DIAGONAL);
            break;
        case 'd': case 'D':
            drone->force_x -= FORCE_MODULE;
            // No change to force_y.
            break;
        case 'f': case 'F':
            // Reset forces.
            drone->force_x = 0;
            drone->force_y = 0;
            break;
        case 'g': case 'G':
            drone->force_x += FORCE_MODULE;
            // No change to force_y.
            break;
        case 'c': case 'C':
            drone->force_x -= FORCE_MODULE / NORM_DIAGONAL;
            drone->force_y += FORCE_MODULE / (2*NORM_DIAGONAL);
            break;
        case 'v': case 'V':
            // No change to force_x; increase force_y.
            drone->force_y += FORCE_MODULE / 2;
            break;
        case 'b': case 'B':
            drone->force_x += FORCE_MODULE / NORM_DIAGONAL;
            drone->force_y += FORCE_MODULE / (2*NORM_DIAGONAL);
            break;
        default:
            break;
    }
}

//---------------------------------------------------------------------
// Signal handler to manage external signals (e.g., from the watchdog).
// Note: Calling non–async-signal-safe functions (like printf, free, fclose, exit)
// within a signal handler may lead to undefined behavior.
void signal_handler(int sig, siginfo_t *info, void *context) {
    (void)context;  // Unused parameter
    if (sig == SIGUSR1) {
        wd_pid = info->si_pid;
        LOG_TO_FILE(debug, "Signal SIGUSR1 received from WATCHDOG");
        kill(wd_pid, SIGUSR1);
    } else if (sig == SIGUSR2) {
        LOG_TO_FILE(debug, "Shutting down by the WATCHDOG");
        printf("Drone shutting down by the WATCHDOG: %d\n", getpid());
        free(obstacles);
        free(targets);
        // Close the log files before exiting.
        fclose(errors);
        fclose(debug);
        exit(EXIT_SUCCESS);
    }
}

//---------------------------------------------------------------------
// Open the shared memory for the drone structure and map it into the process's address space.
int open_drone_shared_memory() {
    int shm_fd = shm_open(DRONE_SHARED_MEMORY, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("[DRONE]: Error opening the drone shared memory");
        LOG_TO_FILE(errors, "Error opening the drone shared memory");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    drone = (Drone *)mmap(0, sizeof(Drone), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (drone == MAP_FAILED) {
        perror("[DRONE]: Error mapping the drone shared memory");
        LOG_TO_FILE(errors, "Error mapping the drone shared memory");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    LOG_TO_FILE(debug, "Opened the drone shared memory");
    return shm_fd;
}

//---------------------------------------------------------------------
// Open the shared memory for the score and map it.
int open_score_shared_memory() {
    int shm_fd = shm_open(SCORE_SHARED_MEMORY, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("[DRONE]: Error opening the score shared memory");
        LOG_TO_FILE(errors, "Error opening the score shared memory");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    score = (Score *)mmap(0, sizeof(Score), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (score == MAP_FAILED) {
        perror("[DRONE]: Error mapping the score shared memory");
        LOG_TO_FILE(errors, "Error mapping the score shared memory");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    LOG_TO_FILE(debug, "Opened the score shared memory");
    return shm_fd;
}

//---------------------------------------------------------------------
// Main loop for the drone process.
// Listens for input from multiple file descriptors (map size, keyboard input,
// obstacle positions, and target positions) using select() and updates the corresponding data.
void drone_process(int mapSizeFd, int inputKeyFd, int obstaclesPosFd, int targetsPosFd) {
    char buffer[256];
    fd_set readFds;
    struct timeval timeout;

    // Determine the maximum file descriptor value for select()
    int maxFd = mapSizeFd;
    if (inputKeyFd > maxFd) maxFd = inputKeyFd;
    if (obstaclesPosFd > maxFd) maxFd = obstaclesPosFd;
    if (targetsPosFd > maxFd) maxFd = targetsPosFd;

    while (1) {
        FD_ZERO(&readFds);
        FD_SET(mapSizeFd, &readFds);
        FD_SET(inputKeyFd, &readFds);
        FD_SET(obstaclesPosFd, &readFds);
        FD_SET(targetsPosFd, &readFds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int activity;
        // Loop in case select() is interrupted by a signal (errno == EINTR)
        do {
            activity = select(maxFd + 1, &readFds, NULL, NULL, &timeout);
        } while(activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("[DRONE]: Error in select()");
            LOG_TO_FILE(errors, "Error in select() for pipe reads");
            break;
        } else if (activity > 0) {
            // Check for updates to the map size.
            if (FD_ISSET(mapSizeFd, &readFds)) {
                ssize_t bytesRead = read(mapSizeFd, buffer, sizeof(buffer) - 1);
                if (bytesRead > 0) {
                    buffer[bytesRead] = '\0';
                    sscanf(buffer, "%d, %d", &game.max_x, &game.max_y);
                }
            }
            // Check for keyboard input.
            if (FD_ISSET(inputKeyFd, &readFds)) {
                ssize_t bytesRead = read(inputKeyFd, buffer, sizeof(buffer) - 1);
                if (bytesRead > 0) {
                    buffer[bytesRead] = '\0';
                    handle_key_pressed(buffer[0], drone);
                }
            }
            // Check for obstacle positions.
            if (FD_ISSET(obstaclesPosFd, &readFds)) {
                ssize_t bytesRead = read(obstaclesPosFd, buffer, sizeof(buffer) - 1);
                if (bytesRead > 0) {
                    buffer[bytesRead] = '\0';
                    char *token = strtok(buffer, "|");
                    int i = 0;
                    // Correct the memset to use the size of Object instead of the pointer size.
                    memset(obstacles, 0, numObstacles * sizeof(Object));
                    while (token != NULL && i < numObstacles) {
                        sscanf(token, "%d,%d,%c,%d", 
                               &obstacles[i].pos_x, &obstacles[i].pos_y, &obstacles[i].type, (int *)&obstacles[i].hit);
                        token = strtok(NULL, "|");
                        i++;
                    }
                }
            }
            // Check for target positions.
            if (FD_ISSET(targetsPosFd, &readFds)) {
                ssize_t bytesRead = read(targetsPosFd, buffer, sizeof(buffer) - 1);
                if (bytesRead > 0) {
                    buffer[bytesRead] = '\0';
                    char *token = strtok(buffer, "|");
                    int i = 0;
                    memset(targets, 0, numTargets * sizeof(Object));
                    while (token != NULL && i < numTargets) {
                        sscanf(token, "%d,%d,%c,%d,%d", 
                               &targets[i].pos_x, &targets[i].pos_y, &targets[i].type, (int *)&targets[i].hit, &targets[i].number);
                        token = strtok(NULL, "|");
                        i++;
                    }
                }
            }
        }
    }

    // Close the file descriptors.
    close(mapSizeFd);
    close(inputKeyFd);
    close(obstaclesPosFd);
    close(targetsPosFd);
}

//---------------------------------------------------------------------
// Main function of the drone process.
int main(int argc, char* argv[]) {
    // OPEN THE LOG FILES in the "logs" folder.
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[DRONE]: Error opening the debug log file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[DRONE]: Error opening the errors log file");
        exit(EXIT_FAILURE);
    }

    // Check for the required number of command line arguments.
    if (argc < 7) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors); 
        exit(EXIT_FAILURE);
    }
    
    LOG_TO_FILE(debug, "Process started");

    // Open the semaphore for child process synchronization.
    sem_t *exec_sem = sem_open("/exec_semaphore", 0);
    if (exec_sem == SEM_FAILED) {
        perror("[DRONE]: Failed to open the semaphore for process synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for process synchronization");
        exit(EXIT_FAILURE);
    }
    sem_post(exec_sem); // Release the semaphore to allow child processes to start.
    sem_close(exec_sem);

    // SETUP THE PIPE FILE DESCRIPTORS.
    int mapSizeFd = atoi(argv[1]);
    int inputKeyFd = atoi(argv[2]);
    int obstaclesPosFd = atoi(argv[3]);
    int targetsPosFd = atoi(argv[4]);
    numObstacles = atoi(argv[5]);
    numTargets = atoi(argv[6]);

    // SETUP THE SIGNAL HANDLERS.
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[DRONE]: Error in sigaction(SIGUSR1)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR1)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[DRONE]: Error in sigaction(SIGUSR2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR2)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Block all signals except SIGUSR1 and SIGUSR2.
    sigset_t sigset;
    sigfillset(&sigset);
    sigdelset(&sigset, SIGUSR1);
    sigdelset(&sigset, SIGUSR2);
    sigprocmask(SIG_SETMASK, &sigset, NULL);

    // Allocate memory for obstacles and targets.
    obstacles = (Object *)malloc(numObstacles * sizeof(Object));
    if (obstacles == NULL) {
        perror("[DRONE]: Error allocating memory for obstacles");
        LOG_TO_FILE(errors, "Error allocating memory for obstacles");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    targets = (Object *)malloc(numTargets * sizeof(Object));
    if (targets == NULL) {
        perror("[DRONE]: Error allocating memory for targets");
        free(obstacles);
        LOG_TO_FILE(errors, "Error allocating memory for targets");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Initialize the allocated memory properly.
    memset(obstacles, 0, numObstacles * sizeof(Object));
    memset(targets, 0, numTargets * sizeof(Object));

    // OPEN THE SHARED MEMORY REGIONS.
    int droneMemFd = open_drone_shared_memory();
    int scoreMemFd = open_score_shared_memory();

    // IMPORT THE INITIAL CONFIGURATION.
    // Wait for 2 seconds to allow the server to send the map size.
    int diff;
    time_t start, finish;
    time(&start);
    do {
        time(&finish);
        diff = difftime(finish, start);
    } while (diff < 2);

    // Read the map size from the server.
    char buffer[50];
    read(mapSizeFd, buffer, sizeof(buffer) - 1);
    sscanf(buffer, "%d, %d", &game.max_x, &game.max_y);
    
    // UPDATE THE DRONE POSITION in a separate thread.
    pthread_t droneThread;
    if (pthread_create(&droneThread, NULL, update_drone_position_thread, NULL) != 0) {
        perror("[DRONE]: Error creating the thread for updating drone position");
        LOG_TO_FILE(errors, "Error creating the thread for updating drone position");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }

    // LAUNCH THE DRONE PROCESS LOOP.
    drone_process(mapSizeFd, inputKeyFd, obstaclesPosFd, targetsPosFd);

    // Wait for the update thread to finish (this point is never reached in the current infinite loop design).
    pthread_join(droneThread, NULL);

    // END PROGRAM:
    // Close the shared memory file descriptors.
    if (close(droneMemFd) == -1 || close(scoreMemFd) == -1) {
        perror("[DRONE]: Error closing shared memory file descriptors");
        LOG_TO_FILE(errors, "Error closing shared memory file descriptors");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Cancel the update thread.
    pthread_cancel(droneThread);

    // Unmap the shared memory regions.
    munmap(drone, sizeof(Drone));
    munmap(score, sizeof(Score));

    free(obstacles);
    free(targets);
    
    fclose(debug);
    fclose(errors);

    return 0;
}
