
#!/bin/bash
PYTHON_PATH="/home/teng/Start-Kit/python"
#0==> all_track, 1==> planner_track, 2==> scheduler_track
# Check if an argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <track>"
    exit 1
fi

# Get the track input
track=$1

# Define the source directory based on the track input
if [ "$track" -eq 0 ]; then
    src=$PYTHON_PATH"/backup/track_all/*"
elif [ "$track" -eq 1 ]; then
    src=$PYTHON_PATH"/backup/track_planner/*"
else
    src=$PYTHON_PATH"/backup/track_scheduler/*"
fi

# Copy the files to the current directory
echo "Copying files from $src to the current folder..."
cp -r $src $PYTHON_PATH

# Notify the user of completion
echo "Files copied successfully!"
