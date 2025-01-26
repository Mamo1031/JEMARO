// Variation 1: process_P Reads the File Descriptor from an Environment Variable
// In this version, the parent sets the file descriptor as an environment variable, which process_P reads from the environment upon execution.

// Parent Process (Setting File Descriptor in Environment Variable)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/wait.h>

int main() {
    int fd[2];
    if (pipe(fd) == -1) {
        perror("pipe failed");
        exit(1);
    }

    pid_t pid = fork();

    if (pid < 0) {
        perror("fork failed");
        exit(1);
    } else if (pid == 0) {
        // Child process
        printf("Child process: PID = %d\n", getpid());

        // Close the writing end of the pipe in the child
        close(fd[1]);

        // Convert fd[0] to a string and set it as an environment variable
        char fd_str[10];
        snprintf(fd_str, sizeof(fd_str), "%d", fd[0]);
        setenv("MY_FD", fd_str, 1);  // Set MY_FD in the environment

        // Execute process_P
        execl("./process_P", "process_P", NULL);

        // If exec fails
        perror("exec failed");
        exit(1);
    } else {
        // Parent process
        printf("Parent process: PID = %d\n", getpid());

        // Close the reading end of the pipe in the parent
        close(fd[0]);

        // Write data to the pipe
        const char *msg = "Hello from parent via environment variable!\n";
        write(fd[1], msg, strlen(msg));

        // Close the writing end of the pipe
        close(fd[1]);

        // Wait for the child to complete
        wait(NULL);
        printf("Child process finished. Parent exiting.\n");
    }

    return 0;
}
