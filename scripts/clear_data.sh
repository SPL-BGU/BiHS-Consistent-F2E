#!/bin/bash

cd "$(dirname "$0")" || exit 1

TARGET_DIR="../data"

# Check if the target directory exists
if [ ! -d "$TARGET_DIR" ]; then
  echo "Error: $TARGET_DIR is not a valid directory."
  exit 1
fi

# Prompt the user for confirmation
read -rp "Are you sure you want to delete all .out files in $TARGET_DIR and its subdirectories? (y/n): " CONFIRMATION

# Check the user's input
if [[ "$CONFIRMATION" != "y" && "$CONFIRMATION" != "Y" ]]; then
  echo "Operation canceled."
  exit 0
fi

# Find and delete all .out files
find "$TARGET_DIR" -type f -name "*.out" -exec rm -f {} \;
