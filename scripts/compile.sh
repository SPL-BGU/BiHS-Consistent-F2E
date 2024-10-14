#!/bin/bash

# Constants
J=8  # Number of parallel jobs for 'make -j'

# Directories
CONFERENCE_DIR="../conference/build/gmake"
JOURNAL_DIR="../journal/build/SFML"

# Functions
usage() {
    echo "Usage: $0 [-c | --conference] [-j | --journal] [--clean] [-h | --help]"
    echo
    echo "Options:"
    echo "  -c, --conference   Only build the conference project"
    echo "  -j, --journal      Only build the journal project"
    echo "  --clean            Run 'make clean' instead of building"
    echo "  -h, --help         Display this help message"
    exit 1
}

build() {
    local dir=$1
    echo "Building in $dir..."
    make -C "$dir" OPENGL=STUB -j $J
}

clean() {
    local dir=$1
    echo "Cleaning in $dir..."
    make -C "$dir" clean
}

# Parse command-line arguments
conference_flag=false
journal_flag=false
clean_flag=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--conference) conference_flag=true ;;
        -j|--journal) journal_flag=true ;;
        --clean) clean_flag=true ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1"; usage ;;
    esac
    shift
done

# Execute based on the flags
if $clean_flag; then
    if $conference_flag; then
        clean "$CONFERENCE_DIR"
    elif $journal_flag; then
        clean "$JOURNAL_DIR"
    else
        clean "$CONFERENCE_DIR"
        clean "$JOURNAL_DIR"
    fi
else
    if $conference_flag; then
        build "$CONFERENCE_DIR"
    elif $journal_flag; then
        build "$JOURNAL_DIR"
    else
        build "$CONFERENCE_DIR"
        build "$JOURNAL_DIR"
    fi
fi
