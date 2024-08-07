#!/bin/bash

# current dir:
#SCRIPT_DIR=$(dirname "$0")
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# swap directories inside this script to ensure data paths are correct:
cd "$script_dir"

TIMESTAMP=$(date +%s%3N)
original="detection"
default_dir="$script_dir/roi"
default="detection_$TIMESTAMP"

# navigate to the current dir:
cd "$SCRIPT_DIR"
# run the first script that creates _.json file
# check if _.json file exists:
if [ -f pixel_data.json ]; then
    # obtain current timestamp:
    
    #mv detection.json "$default.json"
    mv $default_dir/$original.json $default_dir/$default.json
    echo "detection.json to detection_$TIMESTAMP.json"
else
    echo "roi/detection.json not found."
fi