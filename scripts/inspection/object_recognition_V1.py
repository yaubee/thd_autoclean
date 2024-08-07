import os
import tensorflow as tf
from tensorflow import keras
import numpy as np
#from tensorflow.keras.preprocessing import image
#from tensorflow.keras.applications.inception_v3 import preprocess_input
import tensorflow
from keras.preprocessing import image
from keras.applications.inception_v3 import preprocess_input
from PIL import ImageFile
@tf.autograph.experimental.do_not_convert


def get_filepaths_in_dir( dir_path):
    filepath_in_dir = [os.path.join(dir_path, f) for f in os.listdir(dir_path)]
    return filepath_in_dir

def load_and_preprocess_image(image_path):
    img = image.load_img(image_path, target_size=(224, 224))  # InceptionV3 target size
    img = image.img_to_array(img)
    img = np.expand_dims(img, axis=0)
    img = preprocess_input(img) #method from keras library for preprocessing
    return img

def mean(lst):
    return sum(lst)/len(lst)

def object_recognition():

    ImageFile.LOAD_TRUNCATED_IMAGES = True
    
    class_names = [
        "PRT_01", "PRT_02", "PRT_03", "PRT_04", "PRT_07",
        "PRT_11", "PRT_12", "PRT_14", "PRT_18", "PRT_19",
        "PRT_20", "PRT_21", "PRT_24", "PRT_25", "PRT_26"
    ]


    #folder_selected = 'Y:\\TC Hutthurm\\Projekte\\Foerderprojekte\\04_Laufend\\AutoClean_ZIM\\04 Projektdurchfuehrung\\05 Forschungsdaten\\Station Bilderkennung\\Bauteilerkennung\\Test\\Realbilder\\Realbilder_schwarz\\PRT_18'

    #model_path = 'Y:\\TC Hutthurm\\Projekte\\Foerderprojekte\\04_Laufend\\AutoClean_ZIM\\04 Projektdurchfuehrung\\05 Forschungsdaten\\Station Bilderkennung\\Bauteilerkennung\\_Trainierte Modelle\\20231127_allBackgrounds\\val\\Models\\AutoClean_CNN_ObjectRecognition_backgrounds.h5'

    folderpath = os.path.dirname(os.path.abspath(__file__))
    folder_name_images = "images"
    folder_selected=folderpath+"/"+folder_name_images

    file_path_model = 'AutoClean_CNN_ObjectRecognition_backgrounds.h5'
    model_path=folderpath+"/"+file_path_model
    
    model = keras.models.load_model(model_path, compile=False)
    model.compile(loss="categorical_crossentropy", optimizer="adam", metrics=["accuracy"])

    all_file_path = get_filepaths_in_dir(folder_selected)


    pred_prt = []
    acc_lst = []

    for file_path in all_file_path:
        if file_path.lower().endswith(('.jpg','.png','.jpeg')):
            img = load_and_preprocess_image(file_path)
            predictions = model.predict(img)
            

            pred_prt.append(class_names[predictions[0].argmax()]) # list with predicted class names

            acc_lst.append(predictions[0].max())


        else:
            print('File found that is not an image file.')


    avg_class = max(pred_prt, key=pred_prt.count)

    avg_acc = mean(acc_lst)
    
    print('The predicted class is ', avg_class)
    #print()
    #print(f"The mean accuracy is {(100*avg_acc):.2f}%.")


    return avg_class

if __name__ == "__main__":
    object_recognition()


        
