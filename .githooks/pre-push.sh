#!/bin/sh
set -e

build_check() {
    echo "starting pre-push git hook..."

    # Get the list of changed files
    CHANGED_FILES=$(git log -p --branches --not --remotes --name-only)

    # Check if any java files have been changed
    if echo "$CHANGED_FILES" | grep -E --quiet "\.java$"; then
        echo "Detected changed Java files"
        gradlew build
        gradlew.bat spotlessApply
        gradlew build

    else
       echo "No Java file changes detected"
    fi

}

# Gives the script permission
chmod +x .githooks/pre-push.sh
# Runs the command
build_check