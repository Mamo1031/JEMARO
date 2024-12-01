#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/wait.h>

int main() {
    pid_t child1, child2;

    // Create child process 1
    child1 = fork();
    if (child1 == 0) {
        // Child process 1 contents: execute "first"
        char *arg_list1[] = { "./first", NULL };
        execvp(arg_list1[0], arg_list1);
        perror("exec failed for first");
        exit(1);
    }

    // Create child process 2
    child2 = fork();
    if (child2 == 0) {
        // Child process 2 contents: execute "second"
        char *arg_list2[] = { "./second", NULL };
        execvp(arg_list2[0], arg_list2);
        perror("exec failed for second");
        exit(1);
    }

    // The parent process waits for the child process to finish
    wait(NULL);  // Wait for child process 1 to finish
    wait(NULL);  // Wait for child process 2 to finish

    printf("Both first and second processes have finished.\n");
    return 0;
}
