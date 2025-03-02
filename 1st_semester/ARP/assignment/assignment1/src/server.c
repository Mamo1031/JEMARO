#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/select.h>
#include <string.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <errno.h>
#include <pthread.h>
#include "utils.h"

// Global log file pointers
FILE *debug, *errors;

// Global process IDs for various child processes
pid_t wd_pid, map_pid, obs_pid, targ_pid;

// Global pointer to the shared Drone structure and shared score
Drone *drone;
float *score;

// Global timing variable and configuration parameters
time_t start;
int n_obs;  // Number of obstacles
int n_targ; // Number of targets

/**
 * server - Main server loop that relays data between processes.
 *
 * This function monitors several file descriptors (using select) for incoming data
 * from the map, input, obstacle, and target processes. When data is available, it forwards
 * the information to the appropriate destination file descriptors.
 *
 * @param drone_write_size_fd: FD to write map dimensions to the drone.
 * @param drone_write_key_fd: FD to write keyboard input to the drone.
 * @param drone_write_obstacles_fd: FD to write obstacle positions to the drone.
 * @param drone_write_targets_fd: FD to write target positions to the drone.
 * @param input_read_key_fd: FD to read key presses from the input process.
 * @param map_read_size_fd: FD to read the map dimensions from the map process.
 * @param map_write_obstacle_fd: FD to write obstacle positions to the map process.
 * @param map_write_target_fd: FD to write target positions to the map process.
 * @param obstacle_write_size_fd: FD to write the map dimensions to the obstacle process.
 * @param obstacle_read_position_fd: FD to read obstacle positions from the obstacle process.
 * @param target_write_size_fd: FD to write the map dimensions to the target process.
 * @param target_read_position_fd: FD to read target positions from the target process.
 */
void server(int drone_write_size_fd, 
            int drone_write_key_fd, 
            int drone_write_obstacles_fd, 
            int drone_write_targets_fd, 
            int input_read_key_fd, 
            int map_read_size_fd, 
            int map_write_obstacle_fd,
            int map_write_target_fd,
            int obstacle_write_size_fd, 
            int obstacle_read_position_fd, 
            int target_write_size_fd, 
            int target_read_position_fd) {

    char buffer[2048];
    fd_set read_fds;
    struct timeval timeout;

    // Determine the maximum file descriptor among those we need to monitor.
    int max_fd = -1;
    if (map_read_size_fd > max_fd) max_fd = map_read_size_fd;
    if (input_read_key_fd > max_fd) max_fd = input_read_key_fd;
    if (obstacle_read_position_fd > max_fd) max_fd = obstacle_read_position_fd;
    if (target_read_position_fd > max_fd) max_fd = target_read_position_fd;

    // Main server loop
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(input_read_key_fd, &read_fds);
        FD_SET(map_read_size_fd, &read_fds);
        FD_SET(obstacle_read_position_fd, &read_fds);
        FD_SET(target_read_position_fd, &read_fds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int activity;
        // Loop in case select() is interrupted by a signal (errno == EINTR)
        do {
            activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        } while(activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("[SERVER]: Error in select");
            LOG_TO_FILE(errors, "Error in select on pipe reads");
            break;
        } else if (activity > 0) {
            memset(buffer, '\0', sizeof(buffer));
            // If new map dimensions are received from the map process...
            if (FD_ISSET(map_read_size_fd, &read_fds)) {
                ssize_t bytes_read = read(map_read_size_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0'; // Null-terminate the string
                    // Forward the new map dimensions to the drone, obstacle, and target processes.
                    write(drone_write_size_fd, buffer, strlen(buffer));
                    write(obstacle_write_size_fd, buffer, strlen(buffer));
                    write(target_write_size_fd, buffer, strlen(buffer));
                    time(&start);  // Reset the start time for timeouts or scheduling.
                }
            }
            // If a key press is received from the input process, forward it to the drone.
            if (FD_ISSET(input_read_key_fd, &read_fds)) {
                ssize_t bytes_read = read(input_read_key_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    write(drone_write_key_fd, buffer, strlen(buffer));
                }
            }
            // If obstacle positions are received, forward them to the drone and map processes.
            if (FD_ISSET(obstacle_read_position_fd, &read_fds)) {
                ssize_t bytes_read = read(obstacle_read_position_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    write(drone_write_obstacles_fd, buffer, strlen(buffer));
                    write(map_write_obstacle_fd, buffer, strlen(buffer));
                }
            }
            // If target positions are received, forward them to the drone and map processes.
            if (FD_ISSET(target_read_position_fd, &read_fds)) {
                ssize_t bytes_read = read(target_read_position_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    write(drone_write_targets_fd, buffer, strlen(buffer));
                    write(map_write_target_fd, buffer, strlen(buffer));
                }
            }
        }
    }

    // Close all relevant file descriptors.
    close(drone_write_size_fd);
    close(drone_write_key_fd);
    close(drone_write_obstacles_fd);
    close(drone_write_targets_fd);
    close(map_read_size_fd);
    close(map_write_obstacle_fd);
    close(map_write_target_fd);
    close(input_read_key_fd);
    close(obstacle_write_size_fd);
    close(obstacle_read_position_fd);
    close(target_write_size_fd);
    close(target_read_position_fd);
}

/**
 * get_konsole_child - Retrieves the PID of a child process running under a terminal.
 *
 * This function executes a "ps" command to list the child processes of the given terminal PID.
 *
 * @param terminal_pid: The PID of the terminal process.
 * @return The PID of the first child process found.
 */
int get_konsole_child(pid_t terminal_pid) {
    char cmd[100];
    sprintf(cmd, "ps --ppid %d -o pid= 2>/dev/null", terminal_pid);

    FILE *pipe = popen(cmd, "r");
    if (pipe == NULL) {
        perror("[SERVER]: Error opening pipe to retrieve terminal child PID");
        LOG_TO_FILE(errors, "Error opening pipe to retrieve terminal child PID");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    int pid;
    fscanf(pipe, "%d", &pid);
    pclose(pipe);
    return pid;
}

/**
 * signal_handler - Handles signals received by the server process.
 *
 * For SIGUSR1, it updates the watchdog PID and forwards the signal back.
 * For SIGUSR2, it performs a shutdown sequence including killing the map process,
 * unlinking shared memory and semaphores, closing log files, and exiting.
 *
 * @param sig: The signal number.
 * @param info: Pointer to a siginfo_t structure containing signal information.
 * @param context: Unused context pointer.
 */
void signal_handler(int sig, siginfo_t* info, void *context) {
    (void) context;  // Unused parameter

    if (sig == SIGUSR1) {
        wd_pid = info->si_pid;
        LOG_TO_FILE(debug, "Signal SIGUSR1 received from WATCHDOG");
        kill(wd_pid, SIGUSR1);
    }
    if (sig == SIGUSR2) {
        LOG_TO_FILE(debug, "Shutting down by the WATCHDOG");
        printf("Server shutting down by the WATCHDOG: %d\n", getpid());

        // Send SIGUSR2 to the map process to instruct it to shutdown.
        if (kill(map_pid, SIGUSR2) == -1) {
            perror("[SERVER]: Error sending SIGTERM to the MAP process");
            LOG_TO_FILE(errors, "Error sending SIGTERM to the MAP process");
            exit(EXIT_FAILURE);
        }

        // Unlink shared memory segments.
        if (shm_unlink(DRONE_SHARED_MEMORY) == -1) {
            perror("[SERVER]: Error unlinking the drone shared memory");
            LOG_TO_FILE(errors, "Error unlinking the drone shared memory");
            fclose(debug);
            fclose(errors);
            exit(EXIT_FAILURE);
        }
        if (shm_unlink(SCORE_SHARED_MEMORY) == -1) {
            perror("[SERVER]: Error unlinking the score shared memory");
            LOG_TO_FILE(errors, "Error unlinking the score shared memory");
            fclose(debug);
            fclose(errors);
            exit(EXIT_FAILURE);
        }

        // Unlink semaphores.
        sem_unlink("drone_sem");
        sem_unlink("/exec_semaphore");
        sem_unlink("/map_semaphore");
        sem_unlink("/target_semaphore");

        fclose(errors);
        fclose(debug);
        exit(EXIT_SUCCESS);
    }
}

/**
 * create_drone_shared_memory - Creates and maps the shared memory for the drone.
 *
 * This function creates the shared memory segment for the Drone structure,
 * sets its size, and maps it into the process's address space.
 *
 * @return The file descriptor of the drone shared memory.
 */
int create_drone_shared_memory() {
    int drone_mem_fd = shm_open(DRONE_SHARED_MEMORY, O_CREAT | O_RDWR, 0666);
    if (drone_mem_fd == -1) {
        perror("[SERVER]: Error opening the drone shared memory");
        LOG_TO_FILE(errors, "Error opening the drone shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    
    // Set the size of the shared memory segment to the size of a Drone structure.
    if (ftruncate(drone_mem_fd, sizeof(Drone)) == -1) {
        perror("[SERVER]: Error setting the size of the drone shared memory");
        LOG_TO_FILE(errors, "Error setting the size of the drone shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Map the shared memory for the drone.
    drone = (Drone *)mmap(0, sizeof(Drone), PROT_READ | PROT_WRITE, MAP_SHARED, drone_mem_fd, 0);
    if (drone == MAP_FAILED) {
        perror("[SERVER]: Error mapping the drone shared memory");
        LOG_TO_FILE(errors, "Error mapping the drone shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    LOG_TO_FILE(debug, "Created and opened the drone shared memory");
    return drone_mem_fd;
}

/**
 * create_score_shared_memory - Creates and maps the shared memory for the score.
 *
 * This function creates the shared memory segment for the score (a float),
 * sets its size, initializes it to 0, and maps it into the process's address space.
 *
 * @return The file descriptor of the score shared memory.
 */
int create_score_shared_memory() {
    int score_mem_fd = shm_open(SCORE_SHARED_MEMORY, O_CREAT | O_RDWR, 0666);
    if (score_mem_fd == -1) {
        perror("[SERVER]: Error opening the score shared memory");
        LOG_TO_FILE(errors, "Error opening the score shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    
    // Set the size of the shared memory segment to the size of a float.
    if (ftruncate(score_mem_fd, sizeof(float)) == -1) {
        perror("[SERVER]: Error setting the size of the score shared memory");
        LOG_TO_FILE(errors, "Error setting the size of the score shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Map the shared memory for the score.
    score = (float *)mmap(0, sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, score_mem_fd, 0);
    if (score == MAP_FAILED) {
        perror("[SERVER]: Error mapping the score shared memory");
        LOG_TO_FILE(errors, "Error mapping the score shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    *score = 0;
    LOG_TO_FILE(debug, "Created and opened the score shared memory");
    return score_mem_fd;
}

/**
 * send_signal_generation_thread - Thread routine for periodically sending signals.
 *
 * This thread waits for 15 seconds, then sends a SIGTERM signal to the target
 * and obstacle processes (if their PIDs are valid). It repeats this cycle indefinitely.
 *
 * @return NULL.
 */
void *send_signal_generation_thread() {
    time_t finish;
    double diff; 
    // Array containing the PIDs of the target and obstacle processes.
    pid_t pid_array[] = {targ_pid, obs_pid};

    while (1) {
        time(&start);
        // Wait until 15 seconds have elapsed.
        do {
            time(&finish);
            diff = difftime(finish, start);
        } while (diff < 15);

        // Send SIGTERM to both target and obstacle processes.
        for (int i = 0; i < 2; i++) {
            if (pid_array[i] < 0)
                continue;
            
            if (kill(pid_array[i], SIGTERM) == -1) {
                perror("[SERVER]: Error sending SIGTERM to target or obstacle process");
                switch (i) {
                    case 0:
                        LOG_TO_FILE(errors, "Error sending SIGTERM to the TARGET");
                        break;
                    case 1:
                        LOG_TO_FILE(errors, "Error sending SIGTERM to the OBSTACLE");
                        break;
                }
            }
        }
    }
}

/**
 * get_pid_by_command - Retrieves the PID of a process based on its command name.
 *
 * This function uses "ps aux" piped to grep to find a process by name, then parses the output
 * to extract its PID.
 *
 * @param process_name: The name of the process to search for.
 * @return The PID if found; otherwise, -1.
 */
int get_pid_by_command(const char *process_name) {
    char command[256];
    char buffer[1024];
    FILE *pipe;
    int pid = -1;

    snprintf(command, sizeof(command), "ps aux | grep '%s' | grep -v 'grep'", process_name);

    pipe = popen(command, "r");
    if (!pipe) {
        perror("[SERVER]: Error opening pipe to retrieve process PID");
        LOG_TO_FILE(errors, "Error opening pipe to retrieve process PID");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        char user[32], cmd_part[128];
        if (sscanf(buffer, "%s %d %*f %*f %*f %*f %*s %*s %*s %*s %[^\n]", user, &pid, cmd_part) == 3) {
            if (strstr(cmd_part, process_name) != NULL) {
                break;
            }
        }
    }

    pclose(pipe);
    return pid;
}

/**
 * main - Entry point for the server process.
 *
 * The server process is responsible for relaying data between various processes
 * (drone, input, map, obstacle, and target processes) via pipes. It also creates
 * and manages shared memory segments, semaphores, and spawns a thread to periodically
 * send SIGTERM signals to the obstacle and target processes.
 *
 * Command-line arguments (expected at least 15):
 *   argv[1]  - FD for writing drone map size.
 *   argv[2]  - FD for writing drone key.
 *   argv[3]  - FD for reading key input.
 *   argv[4]  - FD for writing obstacle map size.
 *   argv[5]  - FD for reading obstacle positions.
 *   argv[6]  - FD for writing target map size.
 *   argv[7]  - FD for reading target positions.
 *   argv[8]  - FD for writing obstacles to drone.
 *   argv[9]  - FD for writing targets to drone.
 *   argv[10] - Initial drone position as "x,y".
 *   argv[11] - Initial drone velocity as "x,y".
 *   argv[12] - Initial drone force as "x,y".
 *   argv[13] - Number of obstacles.
 *   argv[14] - Number of targets.
 *
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
    /* OPEN THE LOG FILES */
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[SERVER]: Error opening the debug file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[SERVER]: Error opening the errors file");
        exit(EXIT_FAILURE);
    }

    if (argc < 15) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    /* OPEN THE SEMAPHORE FOR CHILD PROCESS SYNCHRONIZATION */
    sem_t *exec_sem = sem_open("/exec_semaphore", 0);
    if (exec_sem == SEM_FAILED) {
        perror("[SERVER]: Failed to open the semaphore for synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for synchronization");
        exit(EXIT_FAILURE);
    }
    sem_post(exec_sem);  // Release semaphore so that child processes can proceed.
    sem_close(exec_sem);

    /* SETUP THE PIPE FILE DESCRIPTORS */
    int drone_write_size_fd = atoi(argv[1]),
        drone_write_key_fd = atoi(argv[2]),
        input_read_key_fd = atoi(argv[3]),
        obstacle_write_size_fd = atoi(argv[4]),
        obstacle_read_position_fd = atoi(argv[5]),
        target_write_size_fd = atoi(argv[6]),
        target_read_position_fd = atoi(argv[7]),
        drone_write_obstacles_fd = atoi(argv[8]),
        drone_write_targets_fd = atoi(argv[9]);

    // Create three pipes for communicating with the map window process.
    int pipe_fd[2], pipe2_fd[2], pipe3_fd[2];
    if (pipe(pipe_fd) == -1) {
        perror("[SERVER]: Error creating pipe for the map");
        LOG_TO_FILE(errors, "Error creating pipe for the map");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(pipe2_fd) == -1) {
        perror("[SERVER]: Error creating pipe 2 for the map");
        LOG_TO_FILE(errors, "Error creating pipe 2 for the map");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(pipe3_fd) == -1) {
        perror("[SERVER]: Error creating pipe 3 for the map");
        LOG_TO_FILE(errors, "Error creating pipe 3 for the map");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    int map_read_size_fd = pipe_fd[0];
    int map_write_obstacle_fd = pipe2_fd[1];
    int map_write_target_fd = pipe3_fd[1];
    char map_write_size_fd_str[10], map_read_obstacle_fd_str[10], map_read_target_fd_str[10];
    snprintf(map_write_size_fd_str, sizeof(map_write_size_fd_str), "%d", pipe_fd[1]);
    snprintf(map_read_obstacle_fd_str, sizeof(map_read_obstacle_fd_str), "%d", pipe2_fd[0]);
    snprintf(map_read_target_fd_str, sizeof(map_read_target_fd_str), "%d", pipe3_fd[0]);

    /* CREATE THE SHARED MEMORY SEGMENTS */
    int drone_mem_fd = create_drone_shared_memory();
    int score_mem_fd = create_score_shared_memory();

    /* CREATE SEMAPHORES */
    sem_unlink("drone_sem");
    drone->sem = sem_open("drone_sem", O_CREAT | O_RDWR, 0666, 1);
    if (drone->sem == SEM_FAILED) {
        perror("[SERVER]: Error creating semaphore for the drone");
        LOG_TO_FILE(errors, "Error creating semaphore for the drone");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Initialize a semaphore for the map process.
    sem_unlink("/map_semaphore");
    sem_t *map_sem = sem_open("/map_semaphore", O_CREAT | O_EXCL, 0666, 0);
    if (map_sem == SEM_FAILED) {
        perror("[SERVER]: Failed to open the semaphore for the map");
        LOG_TO_FILE(errors, "Failed to open the semaphore for the map");
        exit(EXIT_FAILURE);
    }

    // Initialize a semaphore for synchronizing the retrieval of target and obstacle PIDs.
    sem_unlink("/target_semaphore");
    sem_t *target_sem = sem_open("/target_semaphore", O_CREAT | O_EXCL, 0666, 0);
    if (target_sem == SEM_FAILED) {
        perror("[SERVER]: Failed to open the semaphore for obstacle and target");
        LOG_TO_FILE(errors, "Failed to open the semaphore for obstacle and target");
        exit(EXIT_FAILURE);
    }

    /* SET THE INITIAL CONFIGURATION */
    // Lock the drone semaphore to safely set initial values.
    sem_wait(drone->sem);
    LOG_TO_FILE(debug, "Initialized initial drone configuration");
    sscanf(argv[10], "%f,%f", &drone->pos_x, &drone->pos_y);
    sscanf(argv[11], "%f,%f", &drone->vel_x, &drone->vel_y);
    sscanf(argv[12], "%f,%f", &drone->force_x, &drone->force_y);

    n_obs = atoi(argv[13]);
    n_targ = atoi(argv[14]);

    char n_obs_str[10];
    snprintf(n_obs_str, sizeof(n_obs_str), "%d", n_obs);
    char n_targ_str[10];
    snprintf(n_targ_str, sizeof(n_targ_str), "%d", n_targ);

    sem_post(drone->sem);
    sem_close(drone->sem);

    /* LAUNCH THE MAP WINDOW PROCESS */
    // Build the argument list for launching the map window in a terminal (e.g., Konsole)
    char *map_window_path[] = {"konsole", "-e", "./bin/map_window", 
                                 map_write_size_fd_str, map_read_obstacle_fd_str, map_read_target_fd_str, 
                                 n_obs_str, n_targ_str, NULL};
    pid_t konsole_map_pid = fork();
    if (konsole_map_pid < 0) {
        perror("[SERVER]: Error forking the map process");
        LOG_TO_FILE(errors, "Error forking the map process");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    } else if (konsole_map_pid == 0) {
        execvp(map_window_path[0], map_window_path);
        perror("[SERVER]: Failed to execute the map window process");
        LOG_TO_FILE(errors, "Failed to execute the map window process");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    } else {
        // Wait for the map process to signal that it has started
        sem_wait(map_sem);
        map_pid = get_konsole_child(konsole_map_pid);
        sem_close(map_sem);
    }
    
    /* SET SIGNAL HANDLERS */
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);
    // Set handler for SIGUSR1
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[SERVER]: Error in sigaction(SIGUSR1)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR1)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set handler for SIGUSR2
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[SERVER]: Error in sigaction(SIGUSR2)");
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

    // Retrieve the PIDs for the obstacle and target processes.
    sem_wait(target_sem);
    obs_pid = get_pid_by_command("./obstacle");
    targ_pid = get_pid_by_command("./target");
    sem_close(target_sem);

    usleep(50000);

    // Launch a thread that periodically sends SIGTERM to the obstacle and target processes.
    pthread_t server_thread;
    if (pthread_create(&server_thread, NULL, send_signal_generation_thread, NULL) != 0) {
        perror("[SERVER]: Error creating the periodic signal thread");
        LOG_TO_FILE(errors, "Error creating the periodic signal thread");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    /* LAUNCH THE SERVER LOOP */
    server(drone_write_size_fd, 
           drone_write_key_fd, 
           drone_write_obstacles_fd, 
           drone_write_targets_fd, 
           input_read_key_fd, 
           map_read_size_fd, 
           map_write_obstacle_fd, 
           map_write_target_fd,
           obstacle_write_size_fd, 
           obstacle_read_position_fd, 
           target_write_size_fd, 
           target_read_position_fd);

    /* END PROGRAM: Clean up resources */
    if (shm_unlink(DRONE_SHARED_MEMORY) == -1 || shm_unlink(SCORE_SHARED_MEMORY) == -1) {
        perror("[SERVER]: Error unlinking shared memory");
        LOG_TO_FILE(errors, "Error unlinking shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (close(drone_mem_fd) == -1 || close(score_mem_fd) == -1) {
        perror("[SERVER]: Error closing shared memory file descriptors");
        LOG_TO_FILE(errors, "Error closing shared memory file descriptors");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    munmap(drone, sizeof(Drone));
    munmap(score, sizeof(float));

    // Unlink semaphores
    sem_unlink("drone_sem");
    sem_unlink("/exec_semaphore");
    sem_unlink("/map_semaphore");
    sem_unlink("/target_semaphore");

    fclose(debug);
    fclose(errors);

    return 0;
}
