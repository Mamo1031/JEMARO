// Variation 2: Parent Passes the File Descriptor Directly via argc/argv
// In this version, the parent passes the file descriptor as a command-line argument to process_P, which reads it from argv.

// Parent Process (Passing File Descriptor Directly via argc/argv)

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

        // Convert fd[0] to a string to pass as an argument
        char fd_str[10];
        snprintf(fd_str, sizeof(fd_str), "%d", fd[0]);

        // Execute process_P with fd[0] as a command-line argument
        execl("./process_P", "process_P", fd_str, NULL);

        // If exec fails
        perror("exec failed");
        exit(1);
    } else {
        // Parent process
        printf("Parent process: PID = %d\n", getpid());

        // Close the reading end of the pipe in the parent
        close(fd[0]);

        // Write data to the pipe
        const char *msg = "Hello from parent via argc/argv!\n";
        write(fd[1], msg, strlen(msg));

        // Close the writing end of the pipe
        close(fd[1]);

        // Wait for the child to complete
        wait(NULL);
        printf("Child process finished. Parent exiting.\n");
    }

    return 0;
}
