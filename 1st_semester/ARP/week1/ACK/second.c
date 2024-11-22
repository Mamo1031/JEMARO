#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define ACK_MSG "OK"

int main() {
    int fd;

    char *myfifo = "/tmp/myfifo";
    char *ackfifo = "/tmp/ackfifo";
    mkfifo(myfifo, 0666);
    mkfifo(ackfifo, 0666);

    char str1[80], str2[80];
    char format_string[80] = "%d,%d";
    int n1, n2;
    double mean;

    bool quit = false;

    while (1) {
        fd = open(myfifo, O_RDONLY);
        if (fd == -1) {
            perror("encountered error while opening the numbers fifo\n");
            exit(EXIT_FAILURE);
        }
        read(fd, str1, 80);

        /* if the first input char is q, exit  */
        if (str1[0] != 'q') {
            /* read numbers from input line */
            sscanf(str1, format_string, &n1, &n2);
            mean = (n1 + n2) / 2.0;
            printf("mean value is: %f, sum is: %d\n", mean, n1 + n2);
            close(fd);
        } else {
            quit = true;
        }

        sleep(5); // wait to demonstate the ACK feature

        printf("sending ACK...\n");
        fflush(stdout);
        fd = open(ackfifo, O_WRONLY);
        if (fd == -1) {
            perror("encountered error while opening the ACK fifo\n");
            exit(EXIT_FAILURE);
        }
        write(fd, ACK_MSG, strlen(ACK_MSG) + 1);
        close(fd);

        if (quit) {
            exit(EXIT_SUCCESS);
        }
    }
    return 0;
}