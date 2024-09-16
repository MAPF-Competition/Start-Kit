
#!/bin/bash

PYTHON_PATH=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo "SCRIPT_DIR: $SCRIPT_DIR"

# combined,  planner,  scheduler_track
# Check if an argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <combined|planner|scheduler>"
    exit 1
fi

# Get the track input
track=$1

# if directory $PYTHON_PATH/tmp does not exist, create it
if [ ! -d $PYTHON_PATH"/tmp" ]; then
    mkdir $PYTHON_PATH"/tmp"
else # clear the directory
    rm -rf $PYTHON_PATH"/tmp"/*
fi

common_src=$PYTHON_PATH"/common/*"

# Define the source directory based on the track input
if [ "$track" = "combined" ]; then
    planner_src=$PYTHON_PATH"/user_planner/*"
    scheduler_src=$PYTHON_PATH"/user_scheduler/*"
elif [ "$track" = "planner" ]; then
    planner_src=$PYTHON_PATH"/user_planner/*"
    scheduler_src=$PYTHON_PATH"/default_scheduler/*"
elif [ "$track" = "scheduler" ]; then
    planner_src=$PYTHON_PATH"/default_planner/*"
    scheduler_src=$PYTHON_PATH"/user_scheduler/*"
else
    echo "Invalid track input. Please enter 'combined', 'planner', or 'scheduler'."
    exit 1
fi

# Copy the files to the current directory
echo "Copying files from $common_src, $planner_src, $scheduler_src to the tmp folder..."
cp -r $common_src $PYTHON_PATH"/tmp"
cp -r $planner_src $PYTHON_PATH"/tmp"
cp -r $scheduler_src $PYTHON_PATH"/tmp"

# Notify the user of completion
echo "Track set to $track."
