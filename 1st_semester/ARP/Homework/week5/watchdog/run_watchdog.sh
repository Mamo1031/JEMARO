#!/bin/bash

# Delete an existing pipe
rm -f pipe_to_watchdog

# Run the watchdog process in the background
gcc watchdog.c -o watchdog
./watchdog &

# 5 worker processes run in the background
gcc worker.c -o worker
for i in {A..E}; do
    ./worker $i &
done

# Show all processes started
echo "All processes started!"
