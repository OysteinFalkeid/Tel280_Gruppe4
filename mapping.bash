#!/bin/bash

# Trap SIGINT (Ctrl+C) and terminate all background processes
trap "kill 0" SIGINT

# Run processes in the background, silencing their output
bash ./bash_sh/cartographer.bash > /dev/null 2>&1 &
bash ./bash_sh/save_map.bash > /dev/null 2>&1 &
bash ./bash_sh/teleop.bash

# Wait for all background processes to complete
wait