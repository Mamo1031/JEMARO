#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <signal.h>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <errno.h>
#include "utils.h"

// Global log file pointers.
FILE *debug, *errors;

// Global game configuration and shared drone pointer.
Game game;
Drone *drone;

// File descriptor used to send the terminal dimensions (map size) to the server.
int server_write_size_fd;

// Global counts for obstacles and targets.
int n_obs;
int n_targ;

// Pointer to the shared score.
Score *score;

/**
 * draw_outer_box - Draws the outer border on the main screen.
 *
 * This function draws a box around the entire terminal screen and prints
 * the current window dimensions and the score at the top.
 */
void draw_outer_box() {
    attron(COLOR_PAIR(1));  // Use color pair 1 (blue on black)
    box(stdscr, 0, 0);      // Draw a border around stdscr
    mvprintw(0, 1, "Map Display - Score: %.2f", score->score);
    attroff(COLOR_PAIR(1));
    refresh();
}

/**
 * render_obstacles - Renders obstacles on the screen.
 *
 * For each obstacle in the provided array, if its coordinates are positive,
 * an "O" character is printed at that position using color pair 3.
 *
 * @param obstacles: Array of obstacle objects.
 */
void render_obstacles(Object obstacles[]) {
    attron(COLOR_PAIR(3));  // Use color pair 3 (red on black)
    for (int i = 0; i < n_obs; i++) {
        // Skip obstacles with non-positive coordinates.
        if (obstacles[i].pos_y <= 0 || obstacles[i].pos_x <= 0)
            continue;
        mvprintw(obstacles[i].pos_y, obstacles[i].pos_x, "O");
    }
    attroff(COLOR_PAIR(3));
    refresh();
}

/**
 * render_targets - Renders targets on the screen.
 *
 * For each target in the provided array, if its coordinates are positive,
 * a "T" character is printed at that position using color pair 2.
 *
 * @param targets: Array of target objects.
 */
void render_targets(Object targets[]) {
    attron(COLOR_PAIR(2));  // Use color pair 2 (green on black)
    char nb[20];
    for (int i = 0; i < n_targ; i++) {
        if (targets[i].pos_y <= 0 || targets[i].pos_x <= 0)
            continue;
        sprintf(nb, "%d", targets[i].number); // Change the index of the target as a string
        mvprintw(targets[i].pos_y, targets[i].pos_x, "%s", nb);
    }
    attroff(COLOR_PAIR(2));
    refresh();
}

/**
 * render_drone - Renders the drone on the screen.
 *
 * The drone is represented by a "+" character using color pair 4.
 *
 * @param x: X-coordinate of the drone.
 * @param y: Y-coordinate of the drone.
 */
void render_drone(float x, float y) {
    attron(COLOR_PAIR(4));  // Use color pair 4 (yellow on black)
    mvprintw((int)y, (int)x, "+");
    attroff(COLOR_PAIR(4));
    refresh();
}

/**
 * write_to_server - Sends the current window dimensions to the server.
 *
 * The dimensions are formatted as "max_x, max_y" and written to the file
 * descriptor server_write_size_fd.
 */
void write_to_server() {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "%d, %d", game.max_x, game.max_y);
    write(server_write_size_fd, buffer, strlen(buffer));
}

/**
 * resize_window - Handles terminal resizing.
 *
 * This function reinitializes ncurses, retrieves the new terminal dimensions,
 * updates the shared game configuration, sends the new dimensions to the server,
 * and redraws the outer border.
 */
void resize_window() {
    endwin();
    refresh();
    clear();

    // Get new terminal dimensions and store them in the global game structure.
    getmaxyx(stdscr, game.max_y, game.max_x);
    resize_term(game.max_y, game.max_x);

    // Send the updated dimensions to the server.
    write_to_server();

    clear();
    refresh();

    draw_outer_box();
}

/**
 * resize_handler - Signal handler for resizing and shutdown signals.
 *
 * Handles SIGWINCH (terminal resize) by calling resize_window().
 * Handles SIGUSR2 (shutdown signal from the server) by cleaning up and exiting.
 *
 * @param sig: Signal number.
 * @param info: Pointer to siginfo_t (unused except for potential future use).
 * @param context: Unused.
 */
void resize_handler(int sig, siginfo_t *info, void *context) {
    (void)info;    // Unused parameter
    (void)context; // Unused parameter

    if (sig == SIGWINCH) {
        resize_window();
    }
    
    if (sig == SIGUSR2) {
        LOG_TO_FILE(debug, "Shutting down by the SERVER");
        endwin();
        // Close the log files
        fclose(errors);
        fclose(debug);
        exit(EXIT_SUCCESS);
    }
}

/**
 * open_drone_shared_memory - Opens and maps the shared memory for the drone.
 *
 * Opens the shared memory segment defined by DRONE_SHARED_MEMORY in read-only mode
 * and maps it into the process's address space. Exits on failure.
 *
 * @return File descriptor of the shared memory segment.
 */
int open_drone_shared_memory() {
    int drone_mem_fd = shm_open(DRONE_SHARED_MEMORY, O_RDONLY, 0666);
    if (drone_mem_fd == -1) {
        perror("[MAP]: Error opening the drone shared memory");
        LOG_TO_FILE(errors, "Error opening the drone shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    drone = (Drone *)mmap(0, sizeof(Drone), PROT_READ, MAP_SHARED, drone_mem_fd, 0);
    if (drone == MAP_FAILED) {
        perror("[MAP]: Error mapping the drone shared memory");
        LOG_TO_FILE(errors, "Error mapping the drone shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    LOG_TO_FILE(debug, "Opened the drone shared memory");
    return drone_mem_fd;
}

/**
 * open_score_shared_memory - Opens and maps the shared memory for the score.
 *
 * Opens the shared memory segment defined by SCORE_SHARED_MEMORY in read-only mode
 * and maps it into the process's address space. Exits on failure.
 *
 * @return File descriptor of the score shared memory segment.
 */
int open_score_shared_memory() {
    int score_mem_fd = shm_open(SCORE_SHARED_MEMORY, O_RDONLY, 0666);
    if (score_mem_fd == -1) {
        perror("[MAP]: Error opening the score shared memory");
        LOG_TO_FILE(errors, "Error opening the score shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    score = (Score *)mmap(0, sizeof(Score), PROT_READ, MAP_SHARED, score_mem_fd, 0);
    if (score == MAP_FAILED) {
        perror("[MAP]: Error mapping the score shared memory");
        LOG_TO_FILE(errors, "Error mapping the score shared memory");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    LOG_TO_FILE(debug, "Opened the score shared memory");
    return score_mem_fd;
}

/**
 * map_render - Renders the complete map.
 *
 * Clears the screen and redraws the outer border, then renders the drone,
 * obstacles, and targets.
 *
 * @param drone: Pointer to the Drone structure.
 * @param obstacles: Array of obstacle objects.
 * @param targets: Array of target objects.
 */
void map_render(Drone *drone, Object obstacles[], Object targets[]) {
    clear();
    draw_outer_box();
    render_drone(drone->pos_x, drone->pos_y);
    render_obstacles(obstacles);
    render_targets(targets);
}

/**
 * main - Entry point of the map_window process.
 *
 * This program uses ncurses to render a map based on the shared memory values,
 * pipe data from the server, and signal handling for terminal resize and shutdown.
 *
 * @param argc: Argument count.
 * @param argv: Argument vector.
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
    /* OPEN THE LOG FILES */
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[MAP]: Error opening the debug file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[MAP]: Error opening the errors file");
        exit(EXIT_FAILURE);
    }

    // Ensure sufficient command-line parameters.
    if (argc < 6) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    /* OPEN THE SEMAPHORE FOR SERVER PROCESS SYNCHRONIZATION */
    sem_t *map_sem = sem_open("/map_semaphore", 0);
    if (map_sem == SEM_FAILED) {
        perror("[MAP]: Failed to open the semaphore for synchronization");
        LOG_TO_FILE(errors, "Failed to open the semaphore for synchronization");
        exit(EXIT_FAILURE);
    }
    sem_post(map_sem);
    sem_close(map_sem);

    /* SETUP THE PIPE */
    // The server_write_size_fd is provided as the first command-line argument.
    server_write_size_fd = atoi(argv[1]);
    // The next arguments are file descriptors for receiving obstacle and target data.
    int server_read_obstacle_fd = atoi(argv[2]);
    int server_read_target_fd = atoi(argv[3]);
    // Note: n_obs is set from argv[3] and n_targ from argv[4]. This might be confusing;
    // ensure that the correct arguments are used.
    n_obs = atoi(argv[3]);
    n_targ = atoi(argv[4]);

    /* SETUP NCURSES */
    initscr();
    start_color();
    cbreak();
    noecho();
    curs_set(0);

    // Initialize color pairs.
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_RED, COLOR_BLACK);
    init_pair(4, COLOR_YELLOW, COLOR_BLACK);

    /* SETUP SIGNALS */
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = resize_handler;
    sigemptyset(&sa.sa_mask);
    // Set the signal handler for SIGWINCH (window size change)
    if (sigaction(SIGWINCH, &sa, NULL) == -1) {
        perror("[MAP]: Error in sigaction(SIGWINCH)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGWINCH)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set the signal handler for SIGUSR2 (shutdown signal)
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[MAP]: Error in sigaction(SIGUSR2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR2)");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    // Block all signals except SIGWINCH and SIGUSR2.
    sigset_t sigset;
    sigfillset(&sigset);
    sigdelset(&sigset, SIGWINCH);
    sigdelset(&sigset, SIGUSR2);
    sigprocmask(SIG_SETMASK, &sigset, NULL);

    /* OPEN SHARED MEMORY */
    int drone_mem_fd = open_drone_shared_memory();
    int score_mem_fd = open_score_shared_memory();

    // Retrieve terminal dimensions and store them in the game structure.
    getmaxyx(stdscr, game.max_y, game.max_x);
    // Send the updated dimensions to the server.
    write_to_server();

    // Create arrays for obstacles and targets.
    Object obstacles[n_obs], targets[n_targ];
    memset(obstacles, 0, sizeof(obstacles));
    memset(targets, 0, sizeof(targets));

    char buffer[256];
    fd_set read_fds;
    struct timeval timeout;
    int max_fd = -1;
    if (server_read_obstacle_fd > max_fd) {
        max_fd = server_read_obstacle_fd;
    }
    if (server_read_target_fd > max_fd) {
        max_fd = server_read_target_fd;
    }

    /* MAP RENDERING LOOP */
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(server_read_obstacle_fd, &read_fds);
        FD_SET(server_read_target_fd, &read_fds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;  // 50 milliseconds timeout
        int activity;
        // Use select() to wait for data on either pipe.
        do {
            activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        } while (activity == -1 && errno == EINTR);

        if (activity < 0) {
            perror("[MAP]: Error in select for pipe reads");
            LOG_TO_FILE(errors, "Error in select for pipe reads");
            break;
        } else if (activity > 0) {
            // If data is available from the obstacle pipe, read and parse it.
            if (FD_ISSET(server_read_obstacle_fd, &read_fds)) {
                ssize_t bytes_read = read(server_read_obstacle_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    char *token = strtok(buffer, "|");
                    int i = 0;
                    while (token != NULL) {
                        sscanf(token, "%d,%d,%c", &obstacles[i].pos_x, &obstacles[i].pos_y, &obstacles[i].type);
                        token = strtok(NULL, "|");
                        i++;
                    }
                    map_render(drone, obstacles, targets);
                }
            }
            // If data is available from the target pipe, read and parse it.
            if (FD_ISSET(server_read_target_fd, &read_fds)) {
                ssize_t bytes_read = read(server_read_target_fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    char *token = strtok(buffer, "|");
                    int i = 0;
                    while (token != NULL) {
                        sscanf(token, "%d,%d,%c,%d,%d", &targets[i].pos_x, &targets[i].pos_y, &targets[i].type, (int*)&targets[i].hit, &targets[i].number);
                        token = strtok(NULL, "|");
                        i++;
                    }
                    map_render(drone, obstacles, targets);
                }
            }
        } else {
            // If no new data, simply re-render the map.
            map_render(drone, obstacles, targets);
        }
    }

    /* END PROGRAM */
    endwin();
    // Close the shared memory file descriptors.
    if (close(drone_mem_fd) == -1 || close(score_mem_fd) == -1) {
        perror("[MAP]: Error closing shared memory file descriptors");
        LOG_TO_FILE(errors, "Error closing shared memory file descriptors");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Unmap the shared memory regions.
    munmap(drone, sizeof(Drone));
    munmap(score, sizeof(Score));

    // Close the log files.
    fclose(debug);
    fclose(errors);

    return 0;
}
