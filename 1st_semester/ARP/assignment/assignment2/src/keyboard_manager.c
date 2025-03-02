#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <ncurses.h>
#include <signal.h>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <pthread.h>
#include "utils.h"

// Global ncurses windows for input and info display, plus a grid for key boxes.
WINDOW *input_window, *info_window, *key_windows[3][3]; 

// Global file pointers for log files.
FILE *debug, *errors;

// Pointer to the shared Drone structure.
Drone *drone;

// Process ID of the watchdog process.
pid_t wd_pid;

// Symbols displayed on the keyboard grid.
const char *symbols[3][3] = {
    {"\\", "^", "/"},
    {"<", "F", ">"},
    {"/", "v", "\\"}
};

// Mutex to synchronize access to the info window.
pthread_mutex_t info_window_mutex;  

//---------------------------------------------------------------------
// Function: update_info_window
// Description: Clears and redraws the information window with the current
//              position, velocity, and force data from the drone structure.
//---------------------------------------------------------------------
void update_info_window() {
    werase(info_window);            // Clear the info window
    box(info_window, 0, 0);           // Draw a border around it
    mvwprintw(info_window, 0, 2, "Dynamics Display");

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    // Calculate the middle of the info window for layout purposes.
    int middle_col = cols / 4;
    int middle_row = rows / 4;
    
    // Display drone position.
    mvwprintw(info_window, middle_row - 2, middle_col - 7, "position {");
    mvwprintw(info_window, middle_row - 1, middle_col - 6, "x: %.6f", drone->pos_x);
    mvwprintw(info_window, middle_row, middle_col - 6, "y: %.6f", drone->pos_y);
    mvwprintw(info_window, middle_row + 1, middle_col - 7, "}");
    
    // Display drone velocity.
    mvwprintw(info_window, middle_row + 3, middle_col - 7, "velocity {");
    mvwprintw(info_window, middle_row + 4, middle_col - 6, "x: %.6f", drone->vel_x);
    mvwprintw(info_window, middle_row + 5, middle_col - 6, "y: %.6f", drone->vel_y);
    mvwprintw(info_window, middle_row + 6, middle_col - 7, "}");
    
    // Display drone force.
    mvwprintw(info_window, middle_row + 8, middle_col - 7, "force {");
    mvwprintw(info_window, middle_row + 9, middle_col - 6, "x: %.6f", drone->force_x);
    mvwprintw(info_window, middle_row + 10, middle_col - 6, "y: %.6f", drone->force_y);
    mvwprintw(info_window, middle_row + 11, middle_col - 7, "}");
    
    wrefresh(info_window);          // Refresh the window to show updates
}

//---------------------------------------------------------------------
// Thread Function: update_info_thread
// Description: Continuously updates the info window at a regular interval.
//              Uses a mutex to ensure thread-safe drawing.
//---------------------------------------------------------------------
void *update_info_thread() {
    while (1) {
        pthread_mutex_lock(&info_window_mutex);
        update_info_window();
        pthread_mutex_unlock(&info_window_mutex);
        usleep(50000);  // Sleep for 50 milliseconds
    }
    return NULL;
}

//---------------------------------------------------------------------
// Function: draw_box
// Description: Draws a bordered box in the given window and prints the specified symbol inside it.
// Parameters:
//   win    - Pointer to the ncurses window to draw the box.
//   symbol - The symbol to display inside the box.
//---------------------------------------------------------------------
void draw_box(WINDOW *win, const char *symbol) {
    box(win, 0, 0);
    mvwprintw(win, 1, 2, "%s", symbol);
    wrefresh(win);
}

//---------------------------------------------------------------------
// Function: handle_key_pressed
// Description: Temporarily changes the color of a key box to simulate a key press effect.
// Parameters:
//   win    - The window representing the key box.
//   symbol - The symbol displayed in the key box.
//---------------------------------------------------------------------
void handle_key_pressed(WINDOW *win, const char *symbol) {
    wattron(win, COLOR_PAIR(2));  // Turn on the color attribute (green background)
    mvwprintw(win, 1, 2, "%s", symbol);
    wrefresh(win);
    usleep(200000);               // Delay for 0.2 seconds
    wattroff(win, COLOR_PAIR(2)); // Turn off the color attribute
    mvwprintw(win, 1, 2, "%s", symbol);
    wrefresh(win);
}

//---------------------------------------------------------------------
// Function: create_keyboard_window
// Description: Creates the input and info windows, as well as the 3x3 grid of key boxes,
//              and draws the initial keyboard layout.
// Parameters:
//   rows - The total number of rows available in the terminal.
//   cols - The total number of columns available in the terminal.
//---------------------------------------------------------------------
void create_keyboard_window(int rows, int cols) {
    // Create two windows: one for the keyboard input and one for displaying info.
    input_window = newwin(rows, cols / 2, 0, 0);
    info_window = newwin(rows, cols / 2, 0, cols / 2);

    // Calculate starting positions so that the 3x3 grid is centered.
    int start_y = (rows - (BOX_HEIGHT * 3)) / 2;
    int start_x = ((cols / 2) - (BOX_WIDTH * 3)) / 2;

    // Create the 3x3 grid of key windows.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            key_windows[i][j] = derwin(input_window, BOX_HEIGHT, BOX_WIDTH, start_y + i * BOX_HEIGHT, start_x + j * BOX_WIDTH);
            refresh();
            draw_box(key_windows[i][j], symbols[i][j]);
        }
    }

    // Draw borders and labels on the input window.
    box(input_window, 0, 0);
    mvwprintw(input_window, 0, 2, "Input Display");
    mvwprintw(input_window, rows - 7, ((cols / 2) - 30) / 2, "Press 'F' to remove forces");
    mvwprintw(input_window, rows - 5, ((cols / 2) - 30) / 2, "Press 'Q' to Quit");
    mvwprintw(input_window, rows - 3, ((cols / 2) - 30) / 2, "Press 'O' to start Over");
    wrefresh(input_window);
}

//---------------------------------------------------------------------
// Function: resize_windows
// Description: Handles terminal resize events by reinitializing ncurses,
//              refreshing the screen, and recreating the keyboard layout.
//---------------------------------------------------------------------
void resize_windows() {
    endwin();
    refresh();
    clear();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    resize_term(rows, cols);  // Adjust ncurses' internal structures to the new terminal size

    clear();
    refresh();

    create_keyboard_window(rows, cols);
}

//---------------------------------------------------------------------
// Signal Handler: signal_handler
// Description: Handles various signals.
//   - SIGWINCH: Terminal window change; triggers window resize.
//   - SIGUSR1: Receives a signal from the watchdog and passes it on.
//   - SIGUSR2: Initiates shutdown of the program.
// Parameters:
//   sig     - The signal number.
//   info    - Pointer to siginfo_t containing signal information.
//   context - Unused context parameter.
//---------------------------------------------------------------------
void signal_handler(int sig, siginfo_t* info, void *context) {
    (void) context;  // Unused parameter

    if (sig == SIGWINCH) {
        resize_windows();
    }
    if (sig == SIGUSR1) {
        wd_pid = info->si_pid;
        LOG_TO_FILE(debug, "Signal SIGUSR1 received from WATCHDOG");
        kill(wd_pid, SIGUSR1);
    }
    if (sig == SIGUSR2) {
        LOG_TO_FILE(debug, "Shutting down by the WATCHDOG");
        
        // Delete all key windows.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                delwin(key_windows[i][j]);
            }
        }
        // Delete the main input and info windows and end ncurses mode.
        delwin(input_window);
        delwin(info_window);
        endwin();

        // Close the log files.
        fclose(errors);
        fclose(debug);

        exit(EXIT_FAILURE);
    }
}

//---------------------------------------------------------------------
// Function: open_drone_shared_memory
// Description: Opens and maps the shared memory segment for the drone structure
//              in read-only mode. Exits if an error occurs.
// Returns: The file descriptor for the shared memory.
//---------------------------------------------------------------------
int open_drone_shared_memory() {
    int drone_mem_fd = shm_open(DRONE_SHARED_MEMORY, O_RDONLY, 0666);
    if (drone_mem_fd == -1) {
        perror("[INPUT]: Error opening the drone shared memory");
        LOG_TO_FILE(errors, "Error opening the drone shared memory");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    drone = (Drone *)mmap(0, sizeof(Drone), PROT_READ, MAP_SHARED, drone_mem_fd, 0);
    if (drone == MAP_FAILED) {
        perror("[INPUT]: Error mapping the drone shared memory");
        LOG_TO_FILE(errors, "Error mapping the drone shared memory");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    LOG_TO_FILE(debug, "Opened the drone shared memory");
    return drone_mem_fd;
}

//---------------------------------------------------------------------
// Function: restart_program
// Description: Calls the executable which kill all processes and restart the simulation.
//---------------------------------------------------------------------
void restart_program() {
    printf("Request to start over...\n");
    system("./restart.sh");  // Execute the restart.sh script
}

//---------------------------------------------------------------------
// Function: keyboard_manager
// Description: Reads keyboard input using ncurses and writes the pressed key
//              to the server pipe. The loop continues until the user presses 'Q' or 'q'.
// Parameters:
//   server_write_key_fd - File descriptor for writing the key to the server.
//---------------------------------------------------------------------
void keyboard_manager(int server_write_key_fd, char *argv[]) {
    int ch;
    while ((ch = getch()) != 'q' && ch != 'Q') {
        if (ch != EOF) {
            if(ch == 'o' || ch == 'O'){ 
                restart_program();
            }
            else{
            write(server_write_key_fd, &ch, sizeof(ch));
            }
        }
    }

}

//---------------------------------------------------------------------
// Main function for the keyboard manager process.
//---------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // OPEN THE LOG FILES from the "logs" folder.
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[INPUT]: Error opening the debug file");
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[INPUT]: Error opening the errors file");
        exit(EXIT_FAILURE);
    }

    // Check that the required parameter is provided.
    if (argc < 2) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors); 
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    // Open the semaphore for child process synchronization.
    sem_t *exec_sem = sem_open("/exec_semaphore", 0);
    if (exec_sem == SEM_FAILED) {
        perror("[INPUT]: Failed to open the semaphore for the exec");
        LOG_TO_FILE(errors, "Failed to open the semaphore for the exec");
        exit(EXIT_FAILURE);
    }
    sem_post(exec_sem); // Release the semaphore so that child processes can start.
    sem_close(exec_sem);

    // SETUP THE PIPE: server_write_key_fd is passed as a command-line argument.
    int server_write_key_fd = atoi(argv[1]);

    // OPEN THE SHARED MEMORY for the drone structure.
    int drone_mem_fd = open_drone_shared_memory();

    // SETUP NCURSES: Initialize the screen, colors, and input modes.
    initscr();
    start_color();
    cbreak();
    noecho();
    curs_set(0);

    // Initialize color pairs.
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(2, COLOR_BLACK, COLOR_GREEN);

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    create_keyboard_window(rows, cols);

    // SETUP SIGNAL HANDLERS.
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);

    // Set signal handler for terminal resize events.
    if (sigaction(SIGWINCH, &sa, NULL) == -1) {
        perror("[INPUT]: Error in sigaction(SIGWINCH)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGWINCH)");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    // Set signal handler for SIGUSR1.
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[INPUT]: Error in sigaction(SIGUSR1)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR1)");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }
    // Set signal handler for SIGUSR2.
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[INPUT]: Error in sigaction(SIGUSR2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR2)");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }

    // Block all signals except SIGWINCH, SIGUSR1, and SIGUSR2.
    sigset_t sigset;
    sigfillset(&sigset);
    sigdelset(&sigset, SIGWINCH);
    sigdelset(&sigset, SIGUSR1);
    sigdelset(&sigset, SIGUSR2);
    sigprocmask(SIG_SETMASK, &sigset, NULL);
    
    // START THE INFO WINDOW UPDATE THREAD.
    pthread_mutex_init(&info_window_mutex, NULL);
    pthread_t info_thread;
    if (pthread_create(&info_thread, NULL, update_info_thread, NULL) != 0) {
        perror("[INPUT]: Error creating the thread for updating the info window");
        LOG_TO_FILE(errors, "Error creating the thread for updating the info window");
        fclose(debug);
        fclose(errors);   
        exit(EXIT_FAILURE);
    }

    // Launch the input manager loop.
    keyboard_manager(server_write_key_fd, argv);

    // END PROGRAM:
    // Send termination signal to the watchdog.
    kill(wd_pid, SIGUSR2);

    close(server_write_key_fd);

    // Join the info thread and destroy the mutex.
    pthread_join(info_thread, NULL);
    pthread_mutex_destroy(&info_window_mutex);

    // Delete all the key windows.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            delwin(key_windows[i][j]);
        }
    }
    delwin(input_window);
    delwin(info_window);
    endwin();

    // Close the shared memory file descriptor.
    if (close(drone_mem_fd) == -1) {
        perror("[INPUT]: Error closing the file descriptor of shared memory");
        LOG_TO_FILE(errors, "Error closing the file descriptor of shared memory");
        fclose(debug);
        fclose(errors); 
        exit(EXIT_FAILURE);
    }
    // Unmap the shared memory region.
    munmap(drone, sizeof(Drone));

    // Close the log files.
    fclose(debug);
    fclose(errors);
    
    return 0;
}
