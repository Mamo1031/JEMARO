#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <string.h>
#include <time.h>

#define CELL0 0
#define CELL1 1

// Shared memory cells
int cells[2] = {0, 0};

void server(int fd_w0[2], int fd_w1[2], int fd_r0[2], int fd_r1[2]) {
    char buffer[80];
    while (1) {
        // Read from W0 or W1
        read(fd_w0[0], buffer, sizeof(buffer));
        if (strlen(buffer) > 0) {
            cells[CELL0] = atoi(buffer);
            printf("Server: Updated CELL0 to %d\n", cells[CELL0]);
        }

        read(fd_w1[0], buffer, sizeof(buffer));
        if (strlen(buffer) > 0) {
            cells[CELL1] = atoi(buffer);
            printf("Server: Updated CELL1 to %d\n", cells[CELL1]);
        }

        // Notify R0 if CELL0 has changed
        sprintf(buffer, "%d", cells[CELL0]);
        write(fd_r0[1], buffer, strlen(buffer) + 1);

        // Notify R1 if CELL1 has changed
        sprintf(buffer, "%d", cells[CELL1]);
        write(fd_r1[1], buffer, strlen(buffer) + 1);
    }
}

void writer(int fd_write[2], int cell) {
    char buffer[80];
    srand(time(NULL) + getpid());
    while (1) {
        int random_num = rand() % 100; // Generate a random number
        sprintf(buffer, "%d", random_num);
        write(fd_write[1], buffer, strlen(buffer) + 1);
        printf("Writer %d: Sent %d\n", cell, random_num);
        sleep(1); // Simulate delay
    }
}

void reader(int fd_read[2], const char *log_file) {
    char buffer[80];
    FILE *log = fopen(log_file, "w");
    if (!log) {
        perror("Failed to open log file");
        exit(1);
    }

    while (1) {
        read(fd_read[0], buffer, sizeof(buffer));
        fprintf(log, "Read value: %s\n", buffer);
        fflush(log); // Ensure logs are written to file immediately
        printf("Reader: Logged %s to %s\n", buffer, log_file);
        sleep(1); // Simulate delay
    }
}

int main() {
    int fd_w0[2], fd_w1[2], fd_r0[2], fd_r1[2];

    // Create pipes
    pipe(fd_w0);
    pipe(fd_w1);
    pipe(fd_r0);
    pipe(fd_r1);

    pid_t pid_server, pid_w0, pid_w1, pid_r0, pid_r1;

    // Create server process
    if ((pid_server = fork()) == 0) {
        close(fd_w0[1]);
        close(fd_w1[1]);
        close(fd_r0[0]);
        close(fd_r1[0]);
        server(fd_w0, fd_w1, fd_r0, fd_r1);
        exit(0);
    }

    // Create writer W0
    if ((pid_w0 = fork()) == 0) {
        close(fd_w0[0]);
        writer(fd_w0, CELL0);
        exit(0);
    }

    // Create writer W1
    if ((pid_w1 = fork()) == 0) {
        close(fd_w1[0]);
        writer(fd_w1, CELL1);
        exit(0);
    }

    // Create reader R0
    if ((pid_r0 = fork()) == 0) {
        close(fd_r0[1]);
        reader(fd_r0, "logfile0.txt");
        exit(0);
    }

    // Create reader R1
    if ((pid_r1 = fork()) == 0) {
        close(fd_r1[1]);
        reader(fd_r1, "logfile1.txt");
        exit(0);
    }

    // Close unused pipe ends in the parent
    close(fd_w0[0]);
    close(fd_w0[1]);
    close(fd_w1[0]);
    close(fd_w1[1]);
    close(fd_r0[0]);
    close(fd_r0[1]);
    close(fd_r1[0]);
    close(fd_r1[1]);

    // Wait for all child processes
    wait(NULL);
    wait(NULL);
    wait(NULL);
    wait(NULL);
    wait(NULL);

    return 0;
}
