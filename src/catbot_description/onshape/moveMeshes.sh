#!/bin/bash

robot_name="catbot"

# Source and destination directories
src_dir="./$robot_name"
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

# Copy robot.urdf to the appropriate subdirectory
mkdir -p "../urdf/$robot_name"
cp "$src_dir/robot.urdf" "../urdf/$robot_name/$robot_name.urdf"

# Change paths in copied URDF files to match new directory structure
find "../urdf/$robot_name" -name '*.urdf' -print0 | while IFS= read -r -d '' file; do
    # Find and edit instances of mesh filenames
    sed -i 's#<mesh filename="package://\(.*\)/\(.*\)_visual.stl"/>#<mesh filename="package://\1/visual/\2_visual.stl"/>#' "$file"
    sed -i 's#<mesh filename="package://\(.*\)/\(.*\)_collision.stl"/>#<mesh filename="package://\1/collision/\2_collision.stl"/>#' "$file"
done
