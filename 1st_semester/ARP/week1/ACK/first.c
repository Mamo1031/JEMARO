#include <fcntl.h>
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
    mkfifo(myfifo, 0666); // fifo channel
    mkfifo(ackfifo, 0666);

    char input_string[80], send_string[80];
    char ack_string[80];

    while (1) {

        fd = open(myfifo, O_WRONLY);

        if (fd == -1) {
            perror("encountered error while opening the numbers fifo\n");
            exit(EXIT_FAILURE);
        }

        printf(
            "Please, write two integer numbers, separated by commas (,), or q "
            "to quit\n");

        /* to be sure that the previous is executed immediately */
        fflush(stdout);

        /* read a full input line */
        fgets(input_string, 80, stdin);

        write(fd, input_string, strlen(input_string) + 1);
        close(fd);

        printf("wating for the ACK...\n");
        fflush(stdout);

        // reading the ack
        fd = open(ackfifo, O_RDONLY);

        if (fd == -1) {
            perror("encountered error while opening the ACK fifo\n");
            exit(EXIT_FAILURE);
        }

        read(fd, ack_string, sizeof(ACK_MSG));
        close(fd);

        if (strcmp(ack_string, "OK") == 0) {
            printf("ACK received!\n");
        } else {
            printf("ACK message different from the one expected!\n");
        }

        /* if the first input char is q, exit  */
        if (input_string[0] == 'q')
            exit(EXIT_SUCCESS);
    }
    return 0;
}