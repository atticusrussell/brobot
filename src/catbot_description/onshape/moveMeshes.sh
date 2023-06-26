#!/bin/bash

# Source and destination directories
src_dir="./"
dest_dir="../meshes"

# Create destination directory if it doesn't exist
mkdir -p "$dest_dir"

# Create subdirectories for "collision" and "visual" if they don't exist
mkdir -p "$dest_dir/collision"
mkdir -p "$dest_dir/visual"

# Find all STL files with "assm" in the name
find "$src_dir" -name '*assm*.stl' -print0 | while IFS= read -r -d '' file; do
    # Check if filename contains "collision" or "visual" and copy to corresponding subdirectory
    if [[ $file == *"collision"* ]]; then
        cp "$file" "$dest_dir/collision"
    elif [[ $file == *"visual"* ]]; then
        cp "$file" "$dest_dir/visual"
    else
        cp "$file" "$dest_dir"
    fi
done
