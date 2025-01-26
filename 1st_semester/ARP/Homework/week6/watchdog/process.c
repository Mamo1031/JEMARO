#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#define LOG_FILE "watchdog.log"

void log_state() {
    FILE *fp = fopen(LOG_FILE, "a");
    if (fp != NULL) {
        fprintf(fp, "%d %ld\n", getpid(), time(NULL));
        fclose(fp);
    }
}

void handle_signal(int sig) {
    log_state();
}

int main() {
    srand(time(NULL) ^ getpid());
    int delay = 1 + rand() % 3;  // Random period between 1 and 3 seconds

    signal(SIGUSR1, handle_signal);

    printf("Process %d started, running with %d sec interval.\n", getpid(), delay);

    while (1) {
        log_state();
        sleep(delay);
    }

    return 0;
}
