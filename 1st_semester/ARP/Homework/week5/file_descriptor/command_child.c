// Variation 2: Parent Passes the File Descriptor Directly via argc/argv
// In this version, the parent passes the file descriptor as a command-line argument to process_P, which reads it from argv.

// process_P (Reads File Descriptor from argc/argv)
// Here, process_P reads the file descriptor passed as a command-line argument.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <fd>\n", argv[0]);
        exit(1);
    }

    // Convert the argument to an integer file descriptor
    int fd = atoi(argv[1]);

    // Read from the file descriptor
    char buffer[100];
    int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        printf("Process P (PID = %d) received from argc/argv: %s", getpid(), buffer);
    } else {
        perror("read failed");
    }

    // Close the file descriptor
    close(fd);

    return 0;
}