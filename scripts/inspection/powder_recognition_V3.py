from collections import Counter
import cv2
import numpy as np
import os
from scipy.signal import argrelmax
from whittaker_eilers import WhittakerSmoother


def thresh(img):

    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    thresh_min = 250


    h = np.histogram(image.ravel(), bins=256)

    data = h[0]

    whittaker_smoother = WhittakerSmoother(lmbda=2e4, order=10, data_length = len(data))
    smoothed_data = whittaker_smoother.smooth(data)

    maxim = argrelmax(np.array(smoothed_data), order=5, mode='wrap')

    if max(maxim[0]) >= thresh_min:
         #print('Pulver erkannt')
         return 'Powdered'
    else:
        #print('Kein Pulver erkannt')
        return 'Clean'

def get_filepaths_in_dir(dir_path):
    filepath_in_dir = [os.path.join(dir_path, f) for f in os.listdir(dir_path)]
    return filepath_in_dir

def powder_recognition():
    
    #folder_selected = 'Y:\\TC Hutthurm\\Projekte\\Foerderprojekte\\04_Laufend\\AutoClean_ZIM\\04 Projektdurchfuehrung\\05 Forschungsdaten\\Station Bilderkennung\\Pulvererkennung\\Pulvererkennung Realbilder\\Test for CM\\Powdered Real'
    folderpath = os.path.dirname(os.path.abspath(__file__))
    folder_name_images = "images"
    folder_selected=folderpath+"/"+folder_name_images

    all_file_path = get_filepaths_in_dir(folder_selected)

    rec_list = []

   
    for file_path in all_file_path:
        if file_path.lower().endswith(('.jpg','.png','.jpeg')):
            img = cv2.imread(file_path)

            rec_list.append(thresh(img))
            
        else:
            print('File found that is not an image file.')
            
    #prediction = max(rec_list, key=rec_list.count)
    prediction = Counter(rec_list).most_common(2)

    if len(prediction) > 1: # if there are images recognized as powdered
        print('Component was detected as powdered.')

        print(f"The component is {prediction[0][0]} to {100*(prediction[0][1]/len(rec_list))}% and {prediction[1][0]} to {100-(100*(prediction[0][1]/len(rec_list)))}%.")
              
        return 'Powdered'
    
    else:
        print('Component was detected as clean.')
                  
        return 'Clean'

if __name__ == "__main__":
    #powder_recognition()
    pass


















































