#!/bin/bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# swap directories inside this script to ensure data paths are correct:
cd "$script_dir"
# dir to save json file
json_dir="$script_dir/roi"

# Set paths to Darknet files
darknet_path="darknetrun"
config_file="$script_dir/cfg/yolov4-custom.cfg"
#weights_file="$script_dir/weights/yolov4-custom_final.weights"
weights_file="$script_dir/weights/yolov4-custom_last.weights"
data_file="$script_dir/data/obj.data"
#threshold="0.3"

TIMESTAMP=$(date +%s%3N)
output_json="detection_$TIMESTAMP"

echo  "Working directory: $script_dir"
echo "Saving detection file to $json_dir/$output_json.json" 

# Directory containing images to process
image_dir="$script_dir/images/"

# File to store the timestamp of the last iteration
timestamp_file="last_iteration_timestamp.txt"
displayed=false # Set to display the last timestamp when detection performed
last_timestamp=$(cat "$timestamp_file" 2>/dev/null)

# Find the last modified image in the directory
image_file=$(find "$image_dir" -type f -name "*.jpg" -newermt "$last_timestamp" | sort -n | tail -n 1)
echo "$image_file"

# Check if there's a new image to process
if [ -n "$image_file" ]; then
	echo "Processing image: $image_file" | tee -a detection_log
	# Run YOLOv4 detection on the image
	$darknet_path detector test $data_file $config_file $weights_file -ext_output -dont_show -out $script_dir/$output_json.json $image_file > "detailed_detection_log" 2>&1
	#$darknet_path detector test $data_file $config_file $weights_file -ext_output -dont_show -out $output_json.json $image_file > /dev/null 2>&1 | tee -a "detection_log"

	# Update the timestamp file with the current timestamp
	date +"%Y-%m-%d %H:%M:%S" > "$timestamp_file"
	displayed=false
else

if [ "$displayed" = false ]; then
	last_detection_timestamp=$(cat "$timestamp_file" 2>/dev/null)
	echo "last image detection at: $last_detection_timestamp" | tee -a detection_log
	displayed=true
fi

fi
