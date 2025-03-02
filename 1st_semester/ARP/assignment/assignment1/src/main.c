#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdbool.h>
#include <sys/stat.h>
#include "cJSON.h"
#include "utils.h"


// Global file pointers for logging
FILE *debug, *errors;

/**
 * get_terminal_child - Retrieves the process ID (PID) of the child process running on the terminal.
 * 
 * This function executes a system command using "ps" to list the child processes of the given parent PID.
 *
 * @param terminal_pid: The PID of the terminal process (e.g., Konsole).
 * @return The PID of the first child process found.
 */
int get_terminal_child(pid_t terminal_pid) {
    char cmd[100];
    // Build the command to get the PID of the child process of the given terminal.
    sprintf(cmd, "ps --ppid %d -o pid= 2>/dev/null", terminal_pid);

    // Open a pipe to execute the command
    FILE *pipe = popen(cmd, "r");
    if (pipe == NULL) {
        perror("[MAIN]: Error opening pipe to retrieve the child PID from the terminal");
        LOG_TO_FILE(errors, "Error opening pipe to retrieve the child PID from the terminal");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    int pid;
    // Read the PID from the pipe output
    fscanf(pipe, "%d", &pid);

    pclose(pipe);
    return pid;
}

// Create the executable to restart the simulation, if it doesn't exist
void create_restart_script() {
    struct stat buffer;
    if (stat("restart.sh", &buffer) != 0) { // Vérifie si le fichier existe
        FILE *file = fopen("restart.sh", "w");
        if (file == NULL) {
            perror("Error creating restart.sh");
            exit(EXIT_FAILURE);
        }

        // Écriture du script dans le fichier
        fprintf(file, "#!/bin/bash\n\n");
        fprintf(file, "# LOGFILE=\"logs/debug.log\" ## Uncomment echo for feedback\n\n");
        fprintf(file, "# exec >> \"$LOGFILE\" 2>&1\n\n");

        fprintf(file, "# Read the PID of the watchdog\n");
        fprintf(file, "if [ ! -f pids.txt ]; then\n");
        fprintf(file, "    echo \"Error: pids.txt not found!\"\n");
        fprintf(file, "    exit 1\n");
        fprintf(file, "fi\n\n");

        fprintf(file, "WATCHDOG_PID=$(head -n 1 pids.txt)\n\n");
        fprintf(file, "# Send SIGUSR2 to the watchdog to kill all the processes\n");
        fprintf(file, "# echo \"[RESTART] Killing watchdog and all processes...\"\n");
        fprintf(file, "kill -SIGUSR2 $WATCHDOG_PID\n\n");

        fprintf(file, "# Wait for all processes to exit\n");
        fprintf(file, "# echo \"[RESTART] Waiting for all processes to exit...\"\n");
        fprintf(file, "while pgrep -x \"watchdog\" || pgrep -x \"server\" || pgrep -x \"drone\" || \\\n");
        fprintf(file, "      pgrep -x \"obstacle\" || pgrep -x \"target\" || pgrep -x \"input\"; do\n");
        fprintf(file, "    sleep 0.5\n");
        fprintf(file, "done\n\n");

        fprintf(file, "# echo \"[RESTART] All processes terminated.\"\n\n");

        fprintf(file, "# Start over the main\n");
        fprintf(file, "#echo \"[RESTART] Starting over...\"\n");
        fprintf(file, "wmctrl -c \"Konsole\"\n");
        fprintf(file, "konsole -e bash -c \"./bin/main; exec bash\"\n\n");
        fprintf(file, "# Wait a bit to write PIDs\n");
        fprintf(file, "sleep 2\n");

        fclose(file);
        chmod("restart.sh", 0755);
        printf("restart.sh created successfully.\n");
    }
}

int main() {
    create_restart_script();
    /* OPEN THE LOG FILES in the "logs" directory */
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[MAIN]: Error opening the debug log file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[MAIN]: Error opening the errors log file");
        exit(EXIT_FAILURE);
    }

    /* INTRODUCTION MESSAGE */
    char key;
    bool startGame = false;

    printf("\nEnter 's' to start or 'q' to quit: ");
    scanf("%c", &key);
    do {
        switch (key) {
            case 's':
                startGame = true;
                printf("\nYou start the game\n");
                break;
            case 'q':
                printf("\nYou quit the game\n");
                exit(EXIT_SUCCESS);
            default:
                printf("\nInvalid input.\nEnter 's' to start or 'q' to quit: ");
                scanf(" %c", &key);
                break;
        }
    } while (!startGame);

    /* IMPORT CONFIGURATION FROM JSON FILE */
    // Read the configuration file "config/config.json" (the file is expected to contain valid JSON)
    char jsonBuffer[1024];
    FILE *file = fopen("config/config.json", "r");
    if (file == NULL) {
        perror("[MAIN]: Error opening configuration file");
        return EXIT_FAILURE;
    }
    // Read the file contents into jsonBuffer (this code is missing reading logic)
    // POTENTIAL BUG: The content of the file is not actually read into jsonBuffer.
    int len = fread(jsonBuffer, 1, sizeof(jsonBuffer), file); 
    fclose(file);

    cJSON *json = cJSON_Parse(jsonBuffer);
    if (json == NULL) {
        perror("[MAIN]: Error parsing configuration file");
        return EXIT_FAILURE;
    }

    // Retrieve configuration parameters from the JSON object.
    char n_obs[10], n_target[10];
    snprintf(n_obs, sizeof(n_obs), "%d", cJSON_GetObjectItemCaseSensitive(json, "NumObstacles")->valueint);
    snprintf(n_target, sizeof(n_target), "%d", cJSON_GetObjectItemCaseSensitive(json, "NumTargets")->valueint);

    cJSON *initial_position = cJSON_GetObjectItemCaseSensitive(json, "DroneInitialPosition");
    cJSON *position = cJSON_GetObjectItem(initial_position, "Position");
    cJSON *velocity = cJSON_GetObjectItem(initial_position, "Velocity");
    cJSON *force = cJSON_GetObjectItem(initial_position, "Force");

    float pos[2], vel[2], f[2];
    for (int i = 0; i < cJSON_GetArraySize(position); ++i) {
        cJSON *el_pos = cJSON_GetArrayItem(position, i);
        cJSON *el_vel = cJSON_GetArrayItem(velocity, i);
        cJSON *el_force = cJSON_GetArrayItem(force, i);
        if (cJSON_IsNumber(el_pos)) pos[i] = el_pos->valuedouble;
        if (cJSON_IsNumber(el_vel)) vel[i] = el_vel->valuedouble;
        if (cJSON_IsNumber(el_force)) f[i] = el_force->valuedouble;
    }
    cJSON_Delete(json);
    
    // Create string representations of the initial position, velocity, and force.
    char pos_str[20], vel_str[20], force_str[20];
    snprintf(pos_str, sizeof(pos_str), "%f,%f", pos[0], pos[1]);
    snprintf(vel_str, sizeof(vel_str), "%f,%f", vel[0], vel[1]);
    snprintf(force_str, sizeof(force_str), "%f,%f", f[0], f[1]);

    /* CREATE PIPES FOR INTER-PROCESS COMMUNICATION */
    int drone_map_fds[2], drone_key_fds[2], input_pipe_fds[2];
    int obstacle_position_fds[2], target_position_fds[2];
    int obstacle_map_fds[2], target_map_fds[2];
    int server_obstacles_fds[2], server_targets_fds[2];

    if (pipe(drone_map_fds) == -1) {
        perror("[MAIN]: Error creating pipe for drone map");
        LOG_TO_FILE(errors, "Error creating pipe for drone map");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(drone_key_fds) == -1) {
        perror("[MAIN]: Error creating pipe for drone key");
        LOG_TO_FILE(errors, "Error creating pipe for drone key");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(input_pipe_fds) == -1) {
        perror("[MAIN]: Error creating pipe for input");
        LOG_TO_FILE(errors, "Error creating pipe for input");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(obstacle_position_fds) == -1) {
        perror("[MAIN]: Error creating pipe for obstacle positions");
        LOG_TO_FILE(errors, "Error creating pipe for obstacle positions");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(obstacle_map_fds) == -1) {
        perror("[MAIN]: Error creating pipe for obstacle map");
        LOG_TO_FILE(errors, "Error creating pipe for obstacle map");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(target_position_fds) == -1) {
        perror("[MAIN]: Error creating pipe for target positions");
        LOG_TO_FILE(errors, "Error creating pipe for target positions");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(target_map_fds) == -1) {
        perror("[MAIN]: Error creating pipe for target map");
        LOG_TO_FILE(errors, "Error creating pipe for target map");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(server_obstacles_fds) == -1) {
        perror("[MAIN]: Error creating pipe for server obstacles");
        LOG_TO_FILE(errors, "Error creating pipe for server obstacles");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    if (pipe(server_targets_fds) == -1) {
        perror("[MAIN]: Error creating pipe for server targets");
        LOG_TO_FILE(errors, "Error creating pipe for server targets");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    /* CONVERT FILE DESCRIPTORS TO STRING REPRESENTATIONS */
    char drone_write_size_fd_str[10], drone_write_key_fd_str[10], input_write_fd_str[10];
    char drone_read_map_fd_str[10], drone_read_key_fd_str[10], input_read_key_fd_str[10];
    char obstacle_write_position_fd_str[10], obstacle_read_position_fd_str[10];
    char target_write_position_fd_str[10], target_read_position_fd_str[10];
    char obstacle_write_size_fd_str[10], obstacle_read_map_fd_str[10];
    char target_write_size_fd_str[10], target_read_map_fd_str[10];
    char drone_write_obstacles_fd_str[10], server_read_obstacles_fd_str[10];
    char drone_write_targets_fd_str[10], server_read_targets_fd_str[10];

    snprintf(obstacle_write_size_fd_str, sizeof(obstacle_write_size_fd_str), "%d", obstacle_map_fds[1]);
    snprintf(target_write_size_fd_str, sizeof(target_write_size_fd_str), "%d", target_map_fds[1]);
    snprintf(obstacle_write_position_fd_str, sizeof(obstacle_write_position_fd_str), "%d", obstacle_position_fds[1]);
    snprintf(target_write_position_fd_str, sizeof(target_write_position_fd_str), "%d", target_position_fds[1]);
    snprintf(drone_write_size_fd_str, sizeof(drone_write_size_fd_str), "%d", drone_map_fds[1]);
    snprintf(drone_write_key_fd_str, sizeof(drone_write_key_fd_str), "%d", drone_key_fds[1]);
    snprintf(input_write_fd_str, sizeof(drone_write_size_fd_str), "%d", input_pipe_fds[1]);
    snprintf(drone_read_map_fd_str, sizeof(drone_read_map_fd_str), "%d", drone_map_fds[0]);
    snprintf(drone_read_key_fd_str, sizeof(drone_read_key_fd_str), "%d", drone_key_fds[0]);
    snprintf(input_read_key_fd_str, sizeof(drone_read_map_fd_str), "%d", input_pipe_fds[0]);
    snprintf(obstacle_read_position_fd_str, sizeof(obstacle_read_position_fd_str), "%d", obstacle_position_fds[0]);
    snprintf(target_read_position_fd_str, sizeof(target_read_position_fd_str), "%d", target_position_fds[0]);
    snprintf(obstacle_read_map_fd_str, sizeof(obstacle_read_map_fd_str), "%d", obstacle_map_fds[0]);
    snprintf(target_read_map_fd_str, sizeof(target_read_map_fd_str), "%d", target_map_fds[0]);
    snprintf(drone_write_obstacles_fd_str, sizeof(drone_write_obstacles_fd_str), "%d", server_obstacles_fds[1]);
    snprintf(server_read_obstacles_fd_str, sizeof(server_read_obstacles_fd_str), "%d", server_obstacles_fds[0]);
    snprintf(drone_write_targets_fd_str, sizeof(drone_write_targets_fd_str), "%d", server_targets_fds[1]);
    snprintf(server_read_targets_fd_str, sizeof(server_read_targets_fd_str), "%d", server_targets_fds[0]);

    /* INITIALIZE SEMAPHORE FOR PROCESS SYNCHRONIZATION */
    sem_unlink("/exec_semaphore");
    sem_t *exec_sem = sem_open("/exec_semaphore", O_CREAT | O_EXCL, 0666, 1);
    if (exec_sem == SEM_FAILED) {
        perror("[MAIN]: Failed to open the semaphore for synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for synchronization");
        exit(EXIT_FAILURE);
    }

    /* LAUNCH THE SERVER AND DRONE PROCESSES */
    pid_t pids[N_PROCS], wd;

    // Array of command-line argument arrays for each process (server, drone, obstacle, target)
    char *inputs[N_PROCS - 1][16] = {
        {"./bin/server", drone_write_size_fd_str, drone_write_key_fd_str, input_read_key_fd_str, obstacle_write_size_fd_str, obstacle_read_position_fd_str, target_write_size_fd_str, target_read_position_fd_str, drone_write_obstacles_fd_str, drone_write_targets_fd_str, pos_str, vel_str, force_str, n_obs, n_target, NULL}, 
        {"./bin/drone", drone_read_map_fd_str, drone_read_key_fd_str, server_read_obstacles_fd_str, server_read_targets_fd_str, n_obs, n_target, NULL},
        {"./bin/obstacle", obstacle_write_position_fd_str, obstacle_read_map_fd_str, n_obs, NULL},
        {"./bin/target", target_write_position_fd_str, target_read_map_fd_str, n_target, n_target, NULL}
    };
    // Launch each process (except the keyboard manager, which is launched later)
    for (int i = 0; i < N_PROCS - 1; i++) {
        sem_wait(exec_sem);  // Wait for synchronization signal
        pids[i] = fork();
        if (pids[i] < 0) {
            perror("[MAIN]: Error during fork");
            LOG_TO_FILE(errors, "Error during fork");
            fclose(debug);
            fclose(errors);
            exit(EXIT_FAILURE);
        } else if (pids[i] == 0) {
            // In child process: execute the corresponding program.
            execvp(inputs[i][0], inputs[i]);
            perror("[MAIN]: Failed to execute process");
            LOG_TO_FILE(errors, "Failed to execute process");
            fclose(debug);
            fclose(errors);
            exit(EXIT_FAILURE);
        }
        usleep(500000); // Wait 0.5 seconds between process launches
    }

    sem_wait(exec_sem); // Wait for the last child process to start

    /* LAUNCH THE KEYBOARD MANAGER */
    pid_t terminalProc = fork();
    // Build the argument array for launching the keyboard manager in Konsole
    char *keyboard_input[] = {"konsole", "-e", "./bin/keyboard_manager", input_write_fd_str, NULL};
    if (terminalProc < 0) {
        perror("[MAIN]: Error forking the keyboard manager");
        LOG_TO_FILE(errors, "Error forking the keyboard manager");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    } else if (terminalProc == 0) {
        execvp(keyboard_input[0], keyboard_input);
        perror("[MAIN]: Failed to execute the keyboard manager");
        LOG_TO_FILE(errors, "Failed to execute the keyboard manager");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    } else {
        sem_wait(exec_sem);  // Wait for the keyboard manager process to start
        pids[N_PROCS - 1] = get_terminal_child(terminalProc);
        // Save PIDs in a file
        FILE *pid_file = fopen("pids.txt", "w");
        if (pid_file == NULL) {
            perror("Failed to open the file with PIDs");
            exit(EXIT_FAILURE);
        }
        for (int i = 0; i < N_PROCS; i++) {
            fprintf(pid_file, "%d\n", pids[i]);
        }
        fclose(pid_file);
        printf("All PIDs were written into pids.txt\n");

    }
    
    usleep(500000); // Short delay

    /* LAUNCH THE WATCHDOG PROCESS */
    char pids_string[N_PROCS][50];
    char *wd_input[N_PROCS + 2];
    wd_input[0] = "./bin/watchdog";
    for (int i = 0; i < N_PROCS; i++) {
        sprintf(pids_string[i], "%d", pids[i]);
        wd_input[i + 1] = pids_string[i];
    }
    wd_input[N_PROCS + 1] = NULL;
    wd = fork();
    if (wd < 0) {
        perror("[MAIN]: Error forking the watchdog process");
        LOG_TO_FILE(errors, "Error forking the watchdog process");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    } else if (wd == 0) {
        execvp(wd_input[0], wd_input);
        perror("[MAIN]: Failed to execute the watchdog process");
        LOG_TO_FILE(errors, "Failed to execute the watchdog process");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    sem_wait(exec_sem); // Wait until the watchdog process has started
    sem_close(exec_sem);
    
    // Wait for all child processes to terminate.
    for (int i = 0; i < N_PROCS + 1; i++) {
        wait(NULL);
    }

    /* END PROGRAM: Clean up resources */
    sem_unlink("/exec_semaphore");

    fclose(debug);
    fclose(errors);

    return 0;
}
