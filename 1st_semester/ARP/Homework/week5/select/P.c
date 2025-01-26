#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#define PIPE1 "pipe_p1_p"
#define PIPE2 "pipe_p2_p"
#define OUTPUT_FILE "output.txt"

int main() {
    int fd1, fd2;
    FILE *output;
    char buffer[1];

    // Create named pipes (if not exists)
    unlink(PIPE1);
    unlink(PIPE2);
    mkfifo(PIPE1, 0666);
    mkfifo(PIPE2, 0666);

    // Open pipes for reading
    fd1 = open(PIPE1, O_RDONLY | O_NONBLOCK);
    fd2 = open(PIPE2, O_RDONLY | O_NONBLOCK);

    if (fd1 == -1 || fd2 == -1) {
        perror("Failed to open pipes");
        exit(1);
    }

    // Open output file
    output = fopen(OUTPUT_FILE, "w");
    if (!output) {
        perror("Failed to open output file");
        exit(1);
    }

    printf("Process P started, waiting for data...\n");

    fd_set read_fds;
    int max_fd = (fd1 > fd2) ? fd1 : fd2;

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(fd1, &read_fds);
        FD_SET(fd2, &read_fds);

        // Use select() to check for available data
        if (select(max_fd + 1, &read_fds, NULL, NULL, NULL) > 0) {
            if (FD_ISSET(fd1, &read_fds)) {
                read(fd1, buffer, 1);
                fwrite(buffer, 1, 1, output);
                fflush(output);
            }
            if (FD_ISSET(fd2, &read_fds)) {
                read(fd2, buffer, 1);
                fwrite(buffer, 1, 1, output);
                fflush(output);
            }
        }
    }

    close(fd1);
    close(fd2);
    fclose(output);
    unlink(PIPE1);
    unlink(PIPE2);

    return 0;
}
