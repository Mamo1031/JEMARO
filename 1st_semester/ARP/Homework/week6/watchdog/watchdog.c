#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <ncurses.h>

#define LOG_FILE "watchdog.log"
#define TIMEOUT 5  // DT seconds (threshold for anomaly detection)

// Array to record PID
pid_t process_pids[3] = {0, 0, 0};

// Signal handler for termination
void terminate_processes(int sig) {
    printf("Watchdog: Terminating all processes...\n");
    for (int i = 0; i < 3; i++) {
        if (process_pids[i] > 0) {
            kill(process_pids[i], SIGTERM);
        }
    }
    exit(0);
}

// Watchdog monitoring loop
void watch_processes() {
    FILE *log_fp;
    time_t last_active[3] = {0, 0, 0};
    
    while (1) {
        sleep(1);
        log_fp = fopen(LOG_FILE, "r");
        if (log_fp == NULL) {
            perror("Failed to open log file");
            continue;
        }
        
        int pid;
        time_t timestamp;
        while (fscanf(log_fp, "%d %ld", &pid, &timestamp) == 2) {
            for (int i = 0; i < 3; i++) {
                if (process_pids[i] == pid) {
                    last_active[i] = timestamp;
                }
            }
        }
        fclose(log_fp);

        time_t now = time(NULL);
        for (int i = 0; i < 3; i++) {
            if (last_active[i] > 0 && (now - last_active[i]) > TIMEOUT) {
                printf("WARNING: Process %d unresponsive. Terminating all.\n", process_pids[i]);
                terminate_processes(SIGTERM);
            }
        }
    }
}

int main() {
    signal(SIGINT, terminate_processes);
    printf("Watchdog started, monitoring processes...\n");
    
    // Fork and start a process
    for (int i = 0; i < 3; i++) {
        process_pids[i] = fork();
        if (process_pids[i] == 0) {
            execl("./process", "./process", NULL);
            exit(0);
        }
    }
    
    watch_processes();
    return 0;
}
