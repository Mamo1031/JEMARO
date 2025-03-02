#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <sys/wait.h>
#include <stdbool.h>
#include "utils.h"

// Global array to hold the PIDs of all monitored processes.
pid_t pids[N_PROCS];

// Global file pointers for the debug and error log files.
FILE *debug, *errors;

// Global array to track the response status of each process.
bool status[N_PROCS];

/**
 * get_current_time - Gets the current time formatted as a string.
 *
 * This function retrieves the current time and formats it as "YYYY-MM-DD HH:MM:SS".
 *
 * @param buffer: The buffer to store the formatted time.
 * @param len: The length of the buffer.
 */
void get_current_time(char *buffer, int len) {
    time_t now = time(NULL);
    strftime(buffer, len, "%Y-%m-%d %H:%M:%S", localtime(&now));
}

/**
 * kill_processes - Sends SIGUSR2 to all monitored processes.
 *
 * Iterates through the global pids array and sends the SIGUSR2 signal to each
 * process. If a kill fails, an error is logged and a message specific to the
 * process type is recorded.
 */
void kill_processes() {
    for (int i = 0; i < N_PROCS; i++) {
        if (kill(pids[i], SIGUSR2) == -1) {
            perror("[WATCHDOG]: Error sending SIGUSR2 to a process");
            switch (i) {
                case 0:
                    LOG_TO_FILE(errors, "Error sending SIGUSR2 to the SERVER");
                    break;
                case 1:
                    LOG_TO_FILE(errors, "Error sending SIGUSR2 to the DRONE");
                    break;
                case 2:
                    LOG_TO_FILE(errors, "Error sending SIGUSR2 to the OBSTACLE");
                    break;
                case 3:
                    LOG_TO_FILE(errors, "Error sending SIGUSR2 to the TARGET");
                    break;
                case 4:
                    LOG_TO_FILE(errors, "Error sending SIGUSR2 to the INPUT");
                    break;
            }
        }
    }
}

/**
 * signal_handler - Handles signals received by the watchdog process.
 *
 * For SIGUSR1, it checks which monitored process sent the signal, logs its
 * receipt, and sets its response status to true.
 * For SIGUSR2, it logs a shutdown message, prints a shutdown notice, calls
 * kill_processes() to terminate all monitored processes, closes log files, and exits.
 * For SIGTERM, it simply logs an "ENDED" message.
 *
 * @param sig: The received signal number.
 * @param info: Pointer to a siginfo_t structure with additional information.
 * @param context: Unused context pointer.
 */
void signal_handler(int sig, siginfo_t* info, void *context) {
    (void) context;  // Unused parameter

    if (sig == SIGUSR1) {
        // Identify the sender process and mark its status as responsive.
        pid_t sender_pid = info->si_pid;
        for (int i = 0; i < N_PROCS; i++) {
            if (sender_pid == pids[i]) {
                switch (i) {
                    case 0:
                        LOG_TO_FILE(debug, "Received signal from the SERVER");
                        break;
                    case 1:
                        LOG_TO_FILE(debug, "Received signal from the DRONE");
                        break;
                    case 2:
                        LOG_TO_FILE(debug, "Received signal from the OBSTACLE");
                        break;
                    case 3:
                        LOG_TO_FILE(debug, "Received signal from the TARGET");
                        break;
                    case 4:
                        LOG_TO_FILE(debug, "Received signal from the INPUT");
                        break;
                }
                status[i] = true;
            }
        }
    }

    if (sig == SIGUSR2) {
        // Received a termination signal (e.g., from the keyboard manager).
        LOG_TO_FILE(debug, "Termination signal received from the keyboard manager; shutting down the drone and server");
        printf("Watchdog shutting down by terminate signal: %d\n", getpid());
        kill_processes();
        fclose(debug);
        fclose(errors);
        exit(EXIT_SUCCESS);
    }

    if (sig == SIGTERM) {
        LOG_TO_FILE(debug, "ENDED");
    }
}

/**
 * watchdog - Main loop of the watchdog process.
 *
 * Periodically sends SIGUSR1 to each monitored process to request an update on
 * its status. After a short wait, it checks whether each process has responded.
 * If any process has not responded within the timeout period, it logs the error,
 * sends a termination signal to all processes, and exits.
 *
 * @param timeout: The timeout value (in seconds) to wait before checking responses.
 */
void watchdog(int timeout) {
    char message[256], current_time[32];
    time_t start, finish;
    double diff;

    while (1) {
        // Request status update from all monitored processes.
        for (int i = 0; i < N_PROCS; i++) {
            status[i] = false;
            if (kill(pids[i], SIGUSR1) == -1) {
                perror("[WATCHDOG]: Error sending SIGUSR1 to a process");
                kill_processes();
                switch (i) {
                    case 0:
                        LOG_TO_FILE(errors, "Error sending SIGUSR1 to the SERVER");
                        break;
                    case 1:
                        LOG_TO_FILE(errors, "Error sending SIGUSR1 to the DRONE");
                        break;
                    case 2:
                        LOG_TO_FILE(errors, "Error sending SIGUSR1 to the OBSTACLE");
                        break;
                    case 3:
                        LOG_TO_FILE(errors, "Error sending SIGUSR1 to the TARGET");
                        break;
                    case 4:
                        LOG_TO_FILE(errors, "Error sending SIGUSR1 to the INPUT");
                        break;
                }
                printf("Watchdog shutting down: %d\n", getpid());
                fclose(debug);
                fclose(errors);
                exit(EXIT_FAILURE);
            }
            /* 
             * Wait 0.5 seconds to allow each process time to respond.
             */
            time(&start);
            do {
                time(&finish);
                diff = difftime(finish, start);
            } while (diff < 0.5);
        }

        // Wait for the full timeout period before checking responses.
        time(&start);
        do {
            time(&finish);
            diff = difftime(finish, start);
        } while (diff < timeout);

        // Check if any process did not respond.
        for (int i = 0; i < N_PROCS; i++) {
            if (!status[i]) {
                get_current_time(current_time, sizeof(current_time));
                switch (i) {
                    case 0:
                        snprintf(message, sizeof(message),
                                 "SERVER process [%d] did not respond or exceeded timeout at %s", pids[i], current_time);
                        LOG_TO_FILE(debug, message);
                        break;
                    case 1:
                        snprintf(message, sizeof(message),
                                 "DRONE process [%d] did not respond or exceeded timeout at %s", pids[i], current_time);
                        LOG_TO_FILE(debug, message);
                        break;
                    case 2:
                        snprintf(message, sizeof(message),
                                 "OBSTACLE process [%d] did not respond or exceeded timeout at %s", pids[i], current_time);
                        LOG_TO_FILE(debug, message);
                        break;
                    case 3:
                        snprintf(message, sizeof(message),
                                 "TARGET process [%d] did not respond or exceeded timeout at %s", pids[i], current_time);
                        LOG_TO_FILE(debug, message);
                        break;
                    case 4:
                        snprintf(message, sizeof(message),
                                 "INPUT process [%d] did not respond or exceeded timeout at %s", pids[i], current_time);
                        LOG_TO_FILE(debug, message);
                        break;
                }
                // If any process fails to respond, terminate all processes.
                kill_processes();
                printf("Watchdog shutting down: %d\n", getpid());
                fclose(debug);
                fclose(errors);
                exit(EXIT_FAILURE);
            }
        }
    }
}

/**
 * main - Entry point of the watchdog process.
 *
 * The watchdog monitors several child processes by periodically sending a status
 * request signal (SIGUSR1) and checking for responses. It also handles termination
 * signals (SIGUSR2 and SIGTERM) and will kill all monitored processes if one fails
 * to respond within a specified timeout period.
 *
 * Command-line arguments: The watchdog expects exactly N_PROCS + 1 arguments.
 *   argv[1] through argv[N_PROCS] should be the PIDs (as strings) of the monitored processes.
 *
 * @param argc: Argument count.
 * @param argv: Argument vector.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    /* OPEN THE LOG FILES */
    debug = fopen("logs/debug.log", "a");
    if (debug == NULL) {
        perror("[WATCHDOG]: Error opening the debug file");
        kill_processes();
        exit(EXIT_FAILURE);
    }
    errors = fopen("logs/errors.log", "a");
    if (errors == NULL) {
        perror("[WATCHDOG]: Error opening the errors file");
        kill_processes();
        exit(EXIT_FAILURE);
    }

    // Check that the number of arguments is exactly N_PROCS + 1.
    if (argc != N_PROCS + 1) {
        LOG_TO_FILE(errors, "Invalid number of parameters");
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debug, "Process started");

    /* OPEN THE SEMAPHORE FOR CHILD PROCESS SYNCHRONIZATION */
    sem_t *exec_sem = sem_open("/exec_semaphore", 0);
    if (exec_sem == SEM_FAILED) {
        perror("[WATCHDOG]: Failed to open the semaphore for exec");
        LOG_TO_FILE(errors, "Failed to open the semaphore for exec");
        exit(EXIT_FAILURE);
    }
    sem_post(exec_sem);  // Release the semaphore so that child processes can proceed.
    sem_close(exec_sem);

    /* SAVE THE CHILD PIDs */
    for (int i = 0; i < N_PROCS; i++) {
        pids[i] = atoi(argv[i+1]);
    }

    /* SET SIGNAL HANDLERS */
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);

    // Set signal handler for SIGUSR1.
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[WATCHDOG]: Error in sigaction(SIGUSR1)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR1)");
        kill_processes();
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set signal handler for SIGUSR2.
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[WATCHDOG]: Error in sigaction(SIGUSR2)");
        LOG_TO_FILE(errors, "Error in sigaction(SIGUSR2)");
        kill_processes();
        fclose(debug);
        fclose(errors);
        exit(EXIT_FAILURE);
    }
    // Set signal handler for SIGTERM.
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("[WATCHDOG]: Error in sigaction(SIGTERM)");
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

    /* LAUNCH THE WATCHDOG LOOP */
    watchdog(TIMEOUT);

    /* END PROGRAM: Close the log files and exit */
    fclose(debug);
    fclose(errors);
    
    return 0;
}
