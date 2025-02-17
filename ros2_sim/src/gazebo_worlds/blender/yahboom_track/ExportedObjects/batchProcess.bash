#!/bin/bash

# Directory containing .blend files
INPUT_DIR="./"
SCRIPT_PATH="./sdf_exporter_batch.py"

for file in "$INPUT_DIR"/*.blend; do
    echo "Processing $file..."
    blender "$file" --background --python "$SCRIPT_PATH"
done
