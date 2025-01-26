#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#define PIPE_NAME "pipe_to_watchdog"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <process_id>\n", argv[0]);
        return 1;
    }

    char process_id = argv[1][0];
    int pipe_fd;
    
    srand(time(NULL) + process_id);

    // open pipe
    pipe_fd = open(PIPE_NAME, O_WRONLY);
    if (pipe_fd == -1) {
        perror("Failed to open named pipe");
        return 1;
    }

    while (1) {
        // Write the process ID to a pipe
        write(pipe_fd, &process_id, 1);
        printf("Process %c sent heartbeat\n", process_id);
        
        // Sleeps randomly for 1 to 3 seconds
        sleep((rand() % 3) + 1);
    }

    close(pipe_fd);
    return 0;
}
