#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>

#define PIPE_NAME "pipe_p1_p2"

pid_t p2_pid;  // Process ID of P2 (for sending signals from P1)
int pipe_fd;
int A, B;  // Input values

// Signal handler for when Ctrl + C is pressed
void handle_sigint(int sig) {
    if (p2_pid > 0) {
        kill(p2_pid, SIGUSR1);  // Send computation request to P2
    }
}

int main() {
    char input[100];

    // Delete the existing pipe if it exists
    unlink(PIPE_NAME);
    mkfifo(PIPE_NAME, 0666);

    // Open the pipe (write-only)
    pipe_fd = open(PIPE_NAME, O_WRONLY);
    if (pipe_fd == -1) {
        perror("Failed to open pipe");
        exit(1);
    }

    printf("P1 started. Enter two numbers (A B) or 'exit' to quit:\n");

    // Set signal handler for Ctrl + C (SIGINT)
    signal(SIGINT, handle_sigint);

    // User input loop
    while (1) {
        printf("Enter A and B: ");
        fgets(input, sizeof(input), stdin);

        // If "exit" is entered, terminate the process
        if (strncmp(input, "exit", 4) == 0) {
            kill(p2_pid, SIGTERM);  // Send termination signal to P2
            break;
        }

        // Retrieve the entered numbers
        if (sscanf(input, "%d %d", &A, &B) == 2) {
            // Send the values to P2 via the pipe
            write(pipe_fd, &A, sizeof(int));
            write(pipe_fd, &B, sizeof(int));

            printf("Sent A=%d, B=%d to P2\n", A, B);
        } else {
            printf("Invalid input. Enter two numbers.\n");
        }
    }

    close(pipe_fd);
    unlink(PIPE_NAME);
    printf("P1 exiting...\n");
    return 0;
}
