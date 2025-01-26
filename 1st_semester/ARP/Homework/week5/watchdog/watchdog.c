#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>

#define PIPE_NAME "pipe_to_watchdog"
#define N_PROCESSES 5
#define TIMEOUT 5  // Monitoring time threshold (seconds)

int main() {
    int pipe_fd;
    char buffer[2];  // 1 character + NULL termination
    time_t last_update[N_PROCESSES] = {0};
    const char process_ids[N_PROCESSES] = {'A', 'B', 'C', 'D', 'E'};
    
    // Create a named pipe (remove if it already exists)
    unlink(PIPE_NAME);
    if (mkfifo(PIPE_NAME, 0666) == -1) {
        perror("Failed to create named pipe");
        return 1;
    }

    // Open the named pipe
    pipe_fd = open(PIPE_NAME, O_RDONLY);
    if (pipe_fd == -1) {
        perror("Failed to open named pipe");
        return 1;
    }

    printf("Watchdog started, monitoring processes...\n");

    while (1) {
        // Read data from the pipe
        ssize_t bytes_read = read(pipe_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';  // Null-terminate the string
            for (int i = 0; i < N_PROCESSES; i++) {
                if (buffer[0] == process_ids[i]) {
                    last_update[i] = time(NULL);  // Record the update time
                }
            }
        }

        // Compare with the current time and warn if a process has not updated for n seconds
        time_t now = time(NULL);
        for (int i = 0; i < N_PROCESSES; i++) {
            if (last_update[i] > 0 && now - last_update[i] > TIMEOUT) {
                printf("WARNING: Process %c has not responded for %d seconds!\n", process_ids[i], TIMEOUT);
                last_update[i] = now;  // Reset once a warning is issued
            }
        }

        sleep(1);  // Monitor every second
    }

    close(pipe_fd);
    unlink(PIPE_NAME);
    return 0;
}
