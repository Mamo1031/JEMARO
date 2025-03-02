#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <bsd/stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <math.h>
#include <sys/select.h>
#include <errno.h>
#include "cJSON.h"
#include "utils.h"
#include <semaphore.h>

// Global log file pointers
FILE *debug, *errors;

// Global game configuration and watchdog process ID
Game game;
pid_t wd_pid;

// Global variable for the number of obstacles; set from command-line argument
int N_OBS;

// File descriptor for writing obstacle positions to the server (initialized to -1)
int obstacle_write_position_fd = -1;

/**
 * generate_obstacles - Creates a set of obstacles with random positions.
 *
 * This function generates N_OBS obstacles with random coordinates within the
 * boundaries defined by game.max_x and game.max_y. Each obstacle is assigned the
 * type 'o' and a "not hit" status. The obstacle data is then formatted as a string
 * and written to the pipe represented by obstacle_write_position_fd.
 */
void generate_obstacles() {
    // Create a local array to hold the generated obstacles
    Object obstacles[N_OBS];
    char obstacleStr[1024] = "";
    char temp[50];

    // Generate obstacles
    for (int i = 0; i < N_OBS; i++) {
        // Generate random coordinates within the game boundaries, leaving a border of 1 unit
        obstacles[i].pos_x = arc4random_uniform(game.max_x - 2) + 1;
        obstacles[i].pos_y = arc4random_uniform(game.max_y - 2) + 1;
        obstacles[i].type = 'o';    // 'o' indicates an obstacle
        obstacles[i].hit = false;   // Initially, the obstacle is not hit

        // Format the obstacle data as "x,y,type,hit"
        // If this is not the last obstacle, append a pipe ('|') as a separator.
        sprintf(temp, (i + 1 != N_OBS) ? "%d,%d,%c,%d|" : "%d,%d,%c,%d",
                obstacles[i].pos_x, obstacles[i].pos_y, obstacles[i].type, (int)obstacles[i].hit);
        strcat(obstacleStr, temp);
    }

    // Write the concatenated obstacle string to the pipe.
    write(obstacle_write_position_fd, obstacleStr, strlen(obstacleStr));

    // Clear the obstacle string buffer
    obstacleStr[0] = '\0';
}

/**
 * signal_handler - Handles external signals for the obstacles process.
 *
 * This handler processes three signals:
 *   - SIGUSR1: Received from the watchdog; forwards the same signal back.
 *   - SIGUSR2: Received when the watchdog requests shutdown; logs and exits.
 *   - SIGTERM: When termination is requested, it regenerates obstacles if the
 *              write file descriptor is set.
 *
 * @param sig: The signal number.
 * @param info: Pointer to a siginfo_t structure with additional signal information.
 * @param context: Unused context pointer.
 */
void signal_handler(int sig, siginfo_t* info, void *context) {
    (void) context;  // Unused parameter

    if (sig == SIGUSR1) {
        // Update watchdog PID and forward SIGUSR1 back to the watchdog.
        wd_pid = info->si_pid;
        LOG_TO_FILE(debug, "Signal SIGUSR1 received from WATCHDOG");
        kill(wd_pid, SIGUSR1);
    }

    if (sig == SIGUSR2) {
        // Log shutdown event and terminate the process.
        LOG_TO_FILE(debug, "Shutting down by the WATCHDOG");
        printf("Obstacle process shutting down by the WATCHDOG: %d\n", getpid());
        fclose(errors);
        fclose(debug);
        exit(EXIT_SUCCESS);
    }

    if (sig == SIGTERM) {
        // On SIGTERM, if the obstacle_write_position_fd is set, generate new obstacles.
        if (obstacle_write_position_fd > 0) {
            LOG_TO_FILE(debug, "Generating new obstacles position");
            generate_obstacles();
        } else {
            // If the file descriptor is not set, output a message.
            printf("NOT SET\n");
        }
    }
}

/**
 * main - Entry point for the obstacles process.
 *
 * This program sets up logging, inter-process communication (pipes and semaphores),
 * signal handling, and then enters a loop to listen for messages on a pipe from the map process.
 * When new map dimensions are received, it regenerates obstacles.
 *
 * @param argc: The number of command-line arguments.
 * @param argv: The argument vector.
 *             Expected arguments:
 *               argv[1] - Pipe file descriptor for writing obstacle positions.
 *               argv[2] - Pipe file descriptor for reading map data.
 *               argv[3] - Number of obstacles.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    // Open log files in the "logs" folder
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[OBSTACLE]: Error opening the debug file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[OBSTACLE]: Error opening the error file");
        exit(EXIT_FAILURE);
    }
    
    // Check that the required command-line arguments are provided.
    if (argc < 3) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    /* OPEN THE SEMAPHORE FOR CHILD PROCESS SYNCHRONIZATION */
    sem_t *exec_sem = sem_open("/exec_semaphore", 0);
    if (exec_sem == SEM_FAILED) {
        perror("[OBSTACLE]: Failed to open the semaphore for synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for synchronization");
        exit(EXIT_FAILURE);
    }
    sem_post(exec_sem); // Release the semaphore so that other processes can proceed.
    sem_close(exec_sem);

    /* SETUP THE PIPE FILE DESCRIPTORS */
    // The first argument is the file descriptor for writing obstacle positions.
    obstacle_write_position_fd = atoi(argv[1]);
    // The second argument is the file descriptor for reading map data.
    int obstacle_read_map_fd = atoi(argv[2]);

    /* IMPORT CONFIGURATION PARAMETERS FROM THE MAIN PROCESS */
    N_OBS = atoi(argv[3]);

    /* SETUP SIGNAL HANDLERS */
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);

    // Set the signal handler for SIGUSR1
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[OBSTACLE]: Error in sigaction(SIGUSR1)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR1)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set the signal handler for SIGUSR2
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[OBSTACLE]: Error in sigaction(SIGUSR2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR2)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set the signal handler for SIGTERM
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("[OBSTACLE]: Error in sigaction(SIGTERM)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGTERM)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Block all signals except SIGUSR1, SIGUSR2, and SIGTERM.
    sigset_t sigset;
    sigfillset(&sigset);
    sigdelset(&sigset, SIGUSR1);
    sigdelset(&sigset, SIGUSR2);
    sigdelset(&sigset, SIGTERM);
    sigprocmask(SIG_SETMASK, &sigset, NULL);
    
    char buffer[256];
    fd_set read_fds;
    struct timeval timeout;

    // Determine the maximum file descriptor for select()
    int max_fd = obstacle_read_map_fd;

    /* MAIN LOOP: Wait for map updates and generate obstacles accordingly */
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(obstacle_read_map_fd, &read_fds);

        // Set a timeout of 1 second for select()
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int activity;
        do {
            activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        } while (activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("[OBSTACLE]: Error in select on pipe");
            LOG_TO_FILE(errors, "Error in select on pipe reads");
            break;
        } else if (activity > 0) {
            // If data is available on the map pipe, read the new dimensions.
            if (FD_ISSET(obstacle_read_map_fd, &read_fds)) {
                ssize_t bytes_read = read(obstacle_read_map_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0'; // Null-terminate the string
                    // Parse the map dimensions from the buffer.
                    sscanf(buffer, "%d, %d", &game.max_x, &game.max_y);
                    // Generate new obstacles based on the new map size.
                    generate_obstacles();
                }
            }
        }
    }

    /* END PROGRAM: Clean up resources */
    fclose(debug);
    fclose(errors);

    return 0;
}
