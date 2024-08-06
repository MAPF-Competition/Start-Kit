
#!/bin/bash

# Check if an argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <track>"
    exit 1
fi

# Get the track input
track=$1

# Define the source directory based on the track input
if [ "$track" -eq 0 ]; then
    src="./backup/track_all/*"
elif [ "$track" -eq 1 ]; then
    src="./backup/track_planner/*"
else
    src="./backup/track_scheduler/*"
fi

# Copy the files to the current directory
echo "Copying files from $src to the current folder..."
cp -r $src .

# Notify the user of completion
echo "Files copied successfully!"
