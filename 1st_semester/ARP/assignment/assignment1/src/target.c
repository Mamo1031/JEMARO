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
#include "utils.h"
#include <semaphore.h>

// Global log file pointers for debugging and error messages.
FILE *debug, *errors;

// Global game configuration structure.
Game game;

// Global watchdog process ID.
pid_t wd_pid;

// Global variable for the number of targets; set via command-line argument.
int N_TARGET;

// File descriptor used for writing target positions to another process.
// Initialized to -1 to indicate an invalid/unset descriptor.
int target_write_position_fd = -1;

/**
 * generate_targets - Generates target objects with random positions.
 *
 * This function creates N_TARGET target objects with random coordinates
 * within the boundaries defined by game.max_x and game.max_y. Each target is
 * assigned a type 't' and a "not hit" flag. The function formats the target data
 * as a string and writes it to the pipe identified by target_write_position_fd.
 */
void generate_targets() {
    int idx_target=1;
    // Create a local array to hold the generated targets.
    Object targets[N_TARGET];
    char targetStr[1024] = "";
    char temp[50];
    
    // Loop through each target.
    for (int i = 0; i < N_TARGET; i++) {
        // Generate random coordinates within the map boundaries,
        // leaving a border of one unit on each side.
        targets[i].pos_x = arc4random_uniform(game.max_x - 2) + 1;
        targets[i].pos_y = arc4random_uniform(game.max_y - 2) + 1;
        targets[i].type = 't';      // 't' indicates a target.
        targets[i].hit = false;     // Initially, the target is not hit.
        targets[i].number = idx_target;
        idx_target++;
        
        // Format the target data as "x,y,type,hit" with a pipe separator for all but the last target.
        sprintf(temp, (i + 1 != N_TARGET) ? "%d,%d,%c,%d,%d|" : "%d,%d,%c,%d,%d", 
                targets[i].pos_x, targets[i].pos_y, targets[i].type, (int)targets[i].hit, targets[i].number);
        strcat(targetStr, temp);
    }
    
    // Write the concatenated target string to the designated pipe.
    write(target_write_position_fd, targetStr, strlen(targetStr));
    // Clear the buffer.
    targetStr[0] = '\0';
}

/**
 * signal_handler - Handles signals received by the target process.
 *
 * This handler processes three signals:
 *   - SIGUSR1: Updates the watchdog PID from the siginfo_t structure and forwards SIGUSR1.
 *   - SIGUSR2: Logs a shutdown message, prints a shutdown message, closes log files, and exits.
 *   - SIGTERM: If target_write_position_fd is valid, calls generate_targets() to regenerate targets.
 *
 * @param sig: The signal number.
 * @param info: Pointer to a siginfo_t structure containing additional signal information.
 * @param context: Unused context pointer.
 */
void signal_handler(int sig, siginfo_t* info, void *context) {
    (void) context;  // Unused parameter

    if (sig == SIGUSR1) {
        // Update watchdog PID and forward SIGUSR1 to it.
        wd_pid = info->si_pid;
        LOG_TO_FILE(debug, "Signal SIGUSR1 received from WATCHDOG");
        kill(wd_pid, SIGUSR1);
    }

    if (sig == SIGUSR2) {
        // Log shutdown, print message, close files, and exit.
        LOG_TO_FILE(debug, "Shutting down by the WATCHDOG");
        printf("Target shutting down by the WATCHDOG: %d\n", getpid());
        fclose(errors);
        fclose(debug);
        exit(EXIT_SUCCESS);
    }

    if (sig == SIGTERM) {
        // On SIGTERM, if the target_write_position_fd is valid, regenerate targets.
        if (target_write_position_fd > 0) {
            LOG_TO_FILE(debug, "Generating new targets position");
            generate_targets();
        }
    }
}

/**
 * main - Entry point for the target process.
 *
 * The target process opens log files, synchronizes with other processes via semaphores,
 * sets up inter-process communication through pipes, installs signal handlers, and then
 * enters a loop waiting for map updates. When a new map size is received, it regenerates targets.
 *
 * Command-line arguments (expected):
 *   argv[1] - File descriptor (as string) for writing target positions.
 *   argv[2] - File descriptor (as string) for reading map data.
 *   argv[3] - Number of targets (as string).
 *
 * @param argc: Number of command-line arguments.
 * @param argv: Argument vector.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    // Open the log files in the "logs" directory.
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[TARGET]: Error opening the debug file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[TARGET]: Error opening the error file");
        exit(EXIT_FAILURE);
    }
    
    // Verify that the required command-line arguments are provided.
    if (argc < 2) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    /* OPEN THE SEMAPHORE FOR CHILD PROCESS SYNCHRONIZATION */
    sem_t *exec_sem = sem_open("/exec_semaphore", 0);
    if (exec_sem == SEM_FAILED) {
        perror("[TARGET]: Failed to open the semaphore for exec synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for exec synchronization");
        exit(EXIT_FAILURE);
    }
    sem_post(exec_sem); // Release the semaphore to allow other child processes to start.
    sem_close(exec_sem);
    
    /* OPEN THE SEMAPHORE FOR SERVER PROCESS SYNCHRONIZATION */
    sem_t *target_sem = sem_open("/target_semaphore", 0);
    if (target_sem == SEM_FAILED) {
        perror("[TARGET]: Failed to open the semaphore for target synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for target synchronization");
        exit(EXIT_FAILURE);
    }
    sem_post(target_sem); // Release the semaphore for the server's command to get this PID.
    sem_close(target_sem); 

    /* SETUP THE PIPE FILE DESCRIPTORS */
    target_write_position_fd = atoi(argv[1]);  // FD for writing target positions.
    int target_read_map_fd = atoi(argv[2]);      // FD for reading map dimensions.

    /* IMPORT CONFIGURATION PARAMETERS FROM THE MAIN PROCESS */
    N_TARGET = atoi(argv[3]);

    /* SETUP SIGNAL HANDLERS */
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);

    // Set signal handler for SIGUSR1.
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[TARGET]: Error in sigaction(SIGUSR1)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR1)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set signal handler for SIGUSR2.
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[TARGET]: Error in sigaction(SIGUSR2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR2)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set signal handler for SIGTERM.
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("[TARGET]: Error in sigaction(SIGTERM)");
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

    int max_fd = -1;
    if (target_read_map_fd > max_fd) {
        max_fd = target_read_map_fd;
    }

    // Main loop: Wait for map updates and regenerate targets as needed.
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(target_read_map_fd, &read_fds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int activity;
        do {
            activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        } while(activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("[TARGET]: Error in select on pipe");
            LOG_TO_FILE(errors, "Error in select on pipe reads");
            break;
        } else if (activity > 0) {
            // If data is received from the map process, parse the map dimensions.
            if (FD_ISSET(target_read_map_fd, &read_fds)) {
                ssize_t bytes_read = read(target_read_map_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0'; // Null-terminate the string
                    sscanf(buffer, "%d, %d", &game.max_x, &game.max_y);
                    // Generate new targets based on the new map size.
                    generate_targets();
                }
            }
        }
    }

    /* END PROGRAM */
    fclose(debug);
    fclose(errors);

    return 0;
}
