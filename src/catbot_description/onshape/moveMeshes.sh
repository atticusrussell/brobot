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

# new name for urdf file
urdf_path="../urdf/$robot_name/"
urdf_suffix="_macro.xacro"
urdf_file="$robot_name$urdf_suffix"
urdf_file_path="$urdf_path$urdf_file"

# Copy robot.urdf to the appropriate subdirectory
mkdir -p "$urdf_path"
cp "$src_dir/robot.urdf" "$urdf_file_path"

# Change paths in copied URDF file to match new directory structure
find "$urdf_file_path" -print0 | while IFS= read -r -d '' file; do
    # Find and edit instances of mesh filenames
    sed -i 's#<mesh filename="package://\(.*\)/\(.*\)_visual.stl"/>#<mesh filename="package://\1/visual/\2_visual.stl"/>#' "$file"
    sed -i 's#<mesh filename="package://\(.*\)/\(.*\)_collision.stl"/>#<mesh filename="package://\1/collision/\2_collision.stl"/>#' "$file"
done

# format the URDF file
# indent
tidy -xml -i -quiet -o $urdf_file_path $urdf_file_path
# re-add  newlines
python3 urdf_newlines.py $urdf_file_path
