/**
 * main.c - User-space application
 *
 * Features:
 *  - J1 (period 300ms), J2 (period 500ms), J3 (period 800ms)
 *  - J4 (aperiodic, background), triggered by J2 (e.g., after J2 runs a few times)
 *
 * Each thread:
 *   (i)   open("/dev/mydriver")
 *   (ii)  write => "[id"
 *   (iii) close
 *   (iv)  waste time (dummy loop)
 *   (v)   open("/dev/mydriver")
 *         write => "id]"
 *         close
 *
 * Compilation: gcc -pthread main.c -o main
 */

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#define PERIOD_J1 300000000  // 300ms in ns
#define PERIOD_J2 500000000  // 500ms in ns
#define PERIOD_J3 800000000  // 800ms in ns

/* Function to waste time - adjust time consumption using loops */
void waste_time(int loops) {
    double val = 0;
    for (int i = 0; i < loops*1000; i++){
        val += i*0.0001;
    }
}

/* Common behavior executed by each thread */
void thread_body(const char *id, long period_ns) {
    struct timespec next_activation;
    clock_gettime(CLOCK_MONOTONIC, &next_activation);

    while (1) {
        /* (i) open */
        int fd = open("/dev/mydriver", O_WRONLY);
        if (fd < 0) {
            perror("open");
            return;
        }
        /* (ii) write "[id" */
        char buf[16];
        snprintf(buf, sizeof(buf), "[%s", id);
        write(fd, buf, strlen(buf));
        /* (iii) close */
        close(fd);

        /* (iv) Waste time */
        waste_time(500);  // Adjust as needed

        /* (v) open => "id]" => close */
        fd = open("/dev/mydriver", O_WRONLY);
        if (fd < 0) {
            perror("open");
            return;
        }
        snprintf(buf, sizeof(buf), "%s]", id);
        write(fd, buf, strlen(buf));
        close(fd);

        /* Sleep until the next period (for periodic tasks) */
        next_activation.tv_nsec += period_ns;
        while (next_activation.tv_nsec >= 1000000000) {
            next_activation.tv_nsec -= 1000000000;
            next_activation.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);
    }
}

/* Global flag: whether to trigger J4 */
volatile int trigger_J4 = 0;

void *threadJ1(void *arg) {
    /* Period 300ms */
    thread_body("1", PERIOD_J1);
    return NULL;
}

void *threadJ3(void *arg) {
    /* Period 800ms */
    thread_body("3", PERIOD_J3);
    return NULL;
}

/* J4: background task (aperiodic) */
void *threadJ4(void *arg) {
    while (1) {
        /* Wait until trigger_J4 is set */
        while (!trigger_J4) {
            usleep(1000);
        }
        trigger_J4 = 0; // Clear flag

        /* Perform actions: "[4" -> waste -> "4]" */
        int fd = open("/dev/mydriver", O_WRONLY);
        if (fd>=0) {
            write(fd, "[4", 2);
            close(fd);
        }
        waste_time(1000); // Adjust as needed
        fd = open("/dev/mydriver", O_WRONLY);
        if (fd>=0) {
            write(fd, "4]", 2);
            close(fd);
        }
    }
    return NULL;
}

/* J2: Period 500ms, triggers J4 at certain intervals */
void *threadJ2(void *arg) {
    struct timespec next_activation;
    clock_gettime(CLOCK_MONOTONIC, &next_activation);

    int counter = 0;

    while (1) {
        /* (i)(ii)(iii) => [2, then waste, then 2] */
        int fd = open("/dev/mydriver", O_WRONLY);
        if (fd>=0) {
            write(fd, "[2", 2);
            close(fd);
        }
        waste_time(500);
        fd = open("/dev/mydriver", O_WRONLY);
        if (fd>=0) {
            write(fd, "2]", 2);
            close(fd);
        }

        /* Trigger J4 every 5th execution of J2 */
        counter++;
        if (counter % 5 == 0) {
            trigger_J4 = 1;  // Trigger J4
        }

        /* Wait for the next period */
        next_activation.tv_nsec += PERIOD_J2;
        while (next_activation.tv_nsec >= 1000000000) {
            next_activation.tv_nsec -= 1000000000;
            next_activation.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);
    }
    return NULL;
}

int main(void) {
    pthread_t th1, th2, th3, th4;

    pthread_create(&th1, NULL, threadJ1, NULL);
    pthread_create(&th2, NULL, threadJ2, NULL);
    pthread_create(&th3, NULL, threadJ3, NULL);
    pthread_create(&th4, NULL, threadJ4, NULL);

    pthread_join(th1, NULL);
    pthread_join(th2, NULL);
    pthread_join(th3, NULL);
    pthread_join(th4, NULL);
    return 0;
}
