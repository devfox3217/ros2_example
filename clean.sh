#!/bin/bash

echo "Cleaning workspace..."

# 삭제할 디렉토리 목록
DIRS=("build" "install" "log")

for dir in "${DIRS[@]}"; do
    if [ -d "$dir" ]; then
        echo "Removing $dir..."
        rm -rf "$dir"
    else
        echo "$dir does not exist."
    fi
done

echo "Clean complete! You can now run 'colcon build' to rebuild the workspace."
