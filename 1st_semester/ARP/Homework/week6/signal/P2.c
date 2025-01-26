#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#define PIPE_NAME "pipe_p1_p2"

int A, B, result = 0;  // Variables used for computation
int pipe_fd;

// Handler for SIGUSR1 (computation request)
void handle_signal(int sig) {
    result = (A + B) + 1;  // Update computation result
    printf("P2: Computed result = %d\n", result);
}

int main() {
    // Execute computation when SIGUSR1 (Ctrl+C in P1) is received
    signal(SIGUSR1, handle_signal);

    // Open the pipe (read-only)
    pipe_fd = open(PIPE_NAME, O_RDONLY);
    if (pipe_fd == -1) {
        perror("Failed to open pipe");
        exit(1);
    }

    printf("P2 started. Waiting for input...\n");

    while (1) {
        // Receive data from P1
        if (read(pipe_fd, &A, sizeof(int)) > 0 && read(pipe_fd, &B, sizeof(int)) > 0) {
            printf("P2 received A=%d, B=%d. Waiting for SIGUSR1...\n", A, B);
        }
    }

    close(pipe_fd);
    printf("P2 exiting...\n");
    return 0;
}
