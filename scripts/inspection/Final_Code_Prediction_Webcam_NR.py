import cv2
import numpy as np
from tensorflow import keras
import os
import time
import matplotlib.pyplot as plt


# Function to prompt user to select a file path
def file_upload():
    from tkinter import Tk
    from tkinter.filedialog import askopenfilename

    # Create the Tkinter root window
    root = Tk()
    root.withdraw()  # Hide the root window

    # Open the file upload dialog
    file_path = askopenfilename()
    return file_path

def folder_path():
    from tkinter import Tk
    from tkinter.filedialog import askdirectory

    # Create the Tkinter root window
    root = Tk()
    root.withdraw()  # Hide the root window

    # Open the file upload dialog
    file_path = askdirectory()
    return file_path

    

# Function to load the trained model
def load_model(model_path):
    model = keras.models.load_model(model_path, compile=False)
    model.compile(loss="categorical_crossentropy", optimizer="adam", metrics=["accuracy"])
    return model

def capture_image(path_to_folder):
    capture = cv2.VideoCapture(0) # 0 for usb-webcam, 1 for in-build webcam

    while True:
        ret, frame = capture.read()

        cv2.imshow("Webcam", frame)

        key = cv2.waitKey(1)
        if key == ord("s"):
            cv2.imwrite(os.path.join(path_to_folder, "Captured_image.jpg"), frame)
            print("Image captured!")
            break

    capture.release()
    cv2.destroyAllWindows()

def webcam_prediction(model, class_names, image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (224, 224))
    image = image / 255.0

    image = np.expand_dims(image, axis=0)
    predictions = model.predict(image)
    class_index = np.argmax(predictions[0])
    class_name = class_names[class_index]
    return class_name

def model_prediction(model_path, file_path):
    img = cv2.imread(file_path)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    print("The Selected Test Image is :")
    time.sleep(2)
    plt.show()

    input_size = (224, 224)

    resized_image = cv2.resize(img, input_size)

    preprocessed_image = resized_image / 255.0

    model = keras.models.load_model(model_path)

    input_image = np.expand_dims(preprocessed_image, axis=0)

    predictions = model.predict(input_image)

    predicted_class = np.argmax(predictions)

    print("Predicted class:", predicted_class)

def main():
    Ask_prompt = "Select the saved model (.h5 File) "
    print(Ask_prompt)
    time.sleep(1)
    model_path = file_upload()

    model = load_model(model_path)

    class_names = [
        "PRT_01",
        "PRT_02",
        "PRT_03",
        "PRT_04",
        "PRT_05",
        "PRT_06",
        "PRT_07",
        "PRT_08",
        "PRT_09",
        "PRT_10",
        "PRT_11",
        "PRT_12",
        "PRT_13",
        "PRT_14",
        "PRT_15",
        "PRT_16",
        "PRT_17",
    ]
    # Open the webcam
    cap = cv2.VideoCapture(0)
    while True:

        ret, frame = cap.read()
        prediction = webcam_prediction(model, class_names, frame)

        # Display the prediction on the frame
        cv2.putText(frame, "Prediction: " + prediction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow('Webcam', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # please comment out the below lines using """ commented code"""" for testing video only prediction


    # Select location of where to store the caputured image from webcam

    """Ask_prompt = "Choose location to save image from Webcam"
    print(Ask_prompt)
    time.sleep(1)
    path_to_folder = folder_path()

    # call capture_image function to open webcam and capture the image
    capture_image(folder_path)

    #Select the captured image

    Ask_prompt = "Select the Test Image path/ Webcame image"
    print(Ask_prompt)
    time.sleep(1)
    file_path = file_upload()
    #Call the model predicion function to get the prediction
    model_prediction(model_path, file_path)"""




main()
