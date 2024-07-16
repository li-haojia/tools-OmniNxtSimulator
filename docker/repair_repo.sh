#!/bin/bash

# Specify the parent directory to search in
PARENT_DIR="/omninxt-sim"

# Find all directories containing a .git directory and add them to safe.directory
find "$PARENT_DIR" -name ".git" | while IFS= read -r gitdir; do
    safedir=$(dirname "$gitdir")
    echo "Adding safe directory: $safedir"
    git config --global --add safe.directory "$safedir"
done

echo "All safe directories have been added."