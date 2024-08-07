import cv2
import json
import threading
import time

class Dot:
    def __init__(self, x, y, pixel_value):
        self.x = x
        self.y = y
        self.pixel_value = pixel_value

    def get_coordinates(self):
        return (self.x, self.y)

    def get_pixel_value(self):
        return self.pixel_value

    def to_dict(self):
        return {"coordinates": [self.x, self.y], "pixel_value": self.pixel_value}

class CameraApp:
    def __init__(self, max_dots=5):
        self.max_dots = max_dots
        self.dots = []
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("Error: Could not open camera.")
        cv2.namedWindow('Camera')
        self.center_dot = Dot(0, 0, (255, 0, 0))  # Blue dot for center

    def add_dot(self, x, y, pixel_value):
        if len(self.dots) >= self.max_dots:
            self.dots.pop(0)
        self.dots.append(Dot(x, y, pixel_value))
        self.store_dot(self.dots[-1])

    def store_dot(self, dot):
        data = dot.to_dict()
        with open('pixel_data.json', 'w') as json_file:
            json.dump(data, json_file)
        print(f"Stored {data} in pixel_data.json")

    def draw_dots(self, frame):
        height, width, _ = frame.shape
        self.center_dot.x, self.center_dot.y = width // 2, height // 2
        cv2.circle(frame, self.center_dot.get_coordinates(), 5, (255, 0, 0), -1)  # Blue center dot

        for dot in self.dots:
            x, y = dot.get_coordinates()
            pixel_value = dot.get_pixel_value()
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Red dot
            text_position = f"({x}, {y})"
            text_value = f"{pixel_value}"
            cv2.putText(frame, text_position, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, text_value, (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            frame = param['frame']
            pixel_value = frame[y, x].tolist()  # Convert to list for JSON serialization
            print(f"Clicked at ({x}, {y}) with pixel value {pixel_value}")
            self.add_dot(x, y, pixel_value)

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            self.draw_dots(frame)

            cv2.imshow('Camera', frame)
            cv2.setMouseCallback('Camera', self.mouse_callback, {'frame': frame})

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def external_function():
    while True:
        # This is an example of a function that runs concurrently with the CameraApp
        print("External function is running.")
        time.sleep(5)

if __name__ == "__main__":
    app = CameraApp()
    
    # Start the CameraApp in a separate thread
    camera_thread = threading.Thread(target=app.run)
    camera_thread.start()

    # Start the external function in the main thread or another thread
    external_function()
    
    # If you want to run the external function in another thread
    # external_thread = threading.Thread(target=external_function)
    # external_thread.start()

    # Wait for the camera thread to finish (optional)
    camera_thread.join()
