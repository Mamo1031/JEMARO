// Variation 1: process_P Reads the File Descriptor from an Environment Variable
// In this version, the parent sets the file descriptor as an environment variable, which process_P reads from the environment upon execution.

// process_P (Reads File Descriptor from Environment Variable)
// This program retrieves the file descriptor from the environment and reads from it.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main() {
    // Get the file descriptor from the environment variable
    char *fd_str = getenv("MY_FD");
    if (fd_str == NULL) {
        fprintf(stderr, "Environment variable MY_FD not found\n");
        exit(1);
    }

    // Convert fd_str to an integer
    int fd = atoi(fd_str);

    // Read from the file descriptor
    char buffer[100];
    int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        printf("Process P (PID = %d) received from environment: %s", getpid(), buffer);
    } else {
        perror("read failed");
    }

    // Close the file descriptor
    close(fd);

    return 0;
}
