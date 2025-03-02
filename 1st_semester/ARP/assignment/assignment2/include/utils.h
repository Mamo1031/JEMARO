#ifndef HELPER_H
#define HELPER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <semaphore.h>
#include <stdbool.h>
#include <time.h>

//---------------------------------------------------------------------
// Macro Definitions
//---------------------------------------------------------------------

// BOX_HEIGHT and BOX_WIDTH define the dimensions for the display box of each key.
#define BOX_HEIGHT 3                            // Height of each key's display box
#define BOX_WIDTH 5                             // Width of each key's display box

// TIMEOUT: Number of seconds after which, if a process does not respond, 
// the watchdog will terminate all processes.
#define TIMEOUT 10                              // Process response timeout (seconds)

// N_PROCS: The number of processes managed by the watchdog.
#define N_PROCS 5                               // Total number of processes monitored by the watchdog

// Shared memory segment names.
#define DRONE_SHARED_MEMORY "/drone_memory"     // Name of the shared memory segment for the drone
#define SCORE_SHARED_MEMORY "/score_memory"     // Name of the shared memory segment for the score

// Physical and simulation parameters.
#define MASS 2                                  // Mass (kg) of the drone
#define FRICTION_COEFFICIENT 0.5                // Friction coefficient of the drone
#define FORCE_MODULE 0.2                        // Incremental force value applied to the drone
#define T 0.5                                   // Time step (delta time) for simulation updates
#define MAX_FREP 15                             // Maximum repulsive force allowed
#define HIT_THR 1.0                             // Hit threshold distance

//---------------------------------------------------------------------
// Data Structure Definitions
//---------------------------------------------------------------------

// Structure representing a drone.
typedef struct {
    float pos_x, pos_y;       // Drone position coordinates (x, y)
    float vel_x, vel_y;       // Drone velocity components (x, y)
    float force_x, force_y;   // Force components currently acting on the drone (x, y)
    sem_t *sem;               // Semaphore for synchronizing access, if needed
} Drone;

// Structure representing an object (such as an obstacle or a target).
typedef struct {
    int pos_x, pos_y;         // Object position coordinates (x, y)
    char type;                // Type identifier (e.g., 'o' for obstacle; other characters for targets)
    bool hit;                 // Flag indicating if the object has been "hit" or activated
    int number;             // Integer displayed to represent the object if it is a target
} Object;

// Structure representing the game boundaries.
typedef struct {
    int max_x, max_y;         // Maximum dimensions of the game area
} Game;

// Structure of the score.
typedef struct {
    int last_target;         // index number of the last target reached by the drone
    float score;             // value of the score
} Score;

//---------------------------------------------------------------------
// Logging Functions and Macros
//---------------------------------------------------------------------

/**
 * writeLog - Writes a log message to the specified file with a timestamp.
 *
 * This function obtains the current time, formats it, locks the file for exclusive access,
 * writes the log message (prefixed with the timestamp), flushes the output, and then unlocks the file.
 *
 * @param file: Pointer to the FILE object where the log message will be written.
 * @param message: The log message to be written.
 */
static inline __attribute__((always_inline)) void writeLog(FILE* file, char* message) {
    char time_now[50];                      // Buffer to store the formatted timestamp
    time_t log_time = time(NULL);           // Get current time
    // Format time as "YYYY-MM-DD HH:MM:SS"
    strftime(time_now, sizeof(time_now), "%Y-%m-%d %H:%M:%S", localtime(&log_time));
    
    // Lock the file exclusively to ensure thread-safe writing.
    int lockResult = flock(fileno(file), LOCK_EX);
    if (lockResult == -1) {
        perror("Failed to lock the log file");
        exit(EXIT_FAILURE);
    }
    
    // Write the log message with timestamp.
    fprintf(file, "[%s] => %s\n", time_now, message);
    fflush(file);

    // Unlock the file.
    int unlockResult = flock(fileno(file), LOCK_UN);
    if (unlockResult == -1) {
        perror("Failed to unlock the log file");
        exit(EXIT_FAILURE);
    }
}

/**
 * LOG_TO_FILE - Macro to log a message with context information.
 *
 * This macro constructs a log message that includes the source file and line number
 * from where the macro is called, then passes the composed message to writeLog().
 *
 * @param file: The FILE pointer to which the log will be written.
 * @param message: The custom message to log.
 */
#define LOG_TO_FILE(file, message) {                                                                                \
    char log[4096];                                                                                                 \
    sprintf(log, "Generated at line [%d] by [%s] with the following message: %s", __LINE__, __FILE__, message);     \
    writeLog(file, log);                                                                                            \
}

#endif
