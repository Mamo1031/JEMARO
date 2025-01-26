#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#define PIPE_NAME "pipe_p2_p"
#define SLEEP_TIME 300000  // Send Interval (microseconds)

int main() {
    // Open the pipe for writing
    int fd = open(PIPE_NAME, O_WRONLY);
    if (fd == -1) {
        perror("Failed to open pipe");
        exit(1);
    }

    while (1) {
        // Send character 'B' to P process
        write(fd, "B", 1);
        usleep(SLEEP_TIME);  // Sleep before sending the next character
    }

    close(fd);
    return 0;
}
