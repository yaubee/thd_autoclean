from tools.processes import run_yolo_on_image, capture_window
import os

folderpath = os.path.dirname(os.path.abspath(__file__))
folder_name_predictions = "predictions" # subdirectory for storing pointclouds changed from data to pcd
folder_name_roi = "roi"
subpath_predictions = folderpath+"/"+folder_name_predictions # when running from terminal / bash script
subpath_roi = folderpath+"/"+folder_name_roi

folderpaths=[subpath_predictions, subpath_roi]
for folderpath_i in folderpaths:
    if not os.path.exists(folderpath_i):
        os.makedirs(folderpath_i)

run_yolo_on_image()
capture_window(subpath_predictions, 'image', max_count=5)
capture_window(subpath_roi, 'json', max_count=5)


