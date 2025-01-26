#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define LOG_FILE "watchdog.log"

void update_display() {
    FILE *fp = fopen(LOG_FILE, "r");
    if (fp == NULL) return;

    clear();
    mvprintw(0, 0, "Process Status Monitor:");
    
    int line = 2;
    int pid;
    time_t timestamp;
    while (fscanf(fp, "%d %ld", &pid, &timestamp) == 2) {
        mvprintw(line++, 0, "PID: %d, Last Active: %ld", pid, timestamp);
    }
    
    fclose(fp);
    refresh();
}

int main() {
    initscr();
    noecho();
    curs_set(FALSE);
    
    while (1) {
        update_display();
        sleep(1);
    }

    endwin();
    return 0;
}
