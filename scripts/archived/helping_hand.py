import cv2

class DisplayCamera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # Open the default camera
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            return

    def run(self):
        while True:
            ret, frame = self.cap.read()  # Capture frame-by-frame
            if not ret:
                print("Error: Failed to capture frame.")
                break

            cv2.imshow('Camera Stream', frame)

            key = cv2.waitKey(1)
            if key == ord('c') or key == ord('C'):
                self.cap.release()
                cv2.destroyAllWindows()
                break

            elif key == ord('q') or key == ord('Q'):
                break

        if key == ord('c') or key == ord('C'):
            capture_camera = CaptureCamera()
            capture_camera.capture_and_show_clicked_point()

class CaptureCamera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # Re-open the default camera
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            return
        _, self.image = self.cap.read()  # Capture frame-by-frame
        self.clicked_point = None

    def capture_and_show_clicked_point(self):
        while True:
            clone = self.image.copy()
            if self.clicked_point:
                cv2.circle(clone, self.clicked_point, 5, (0, 255, 0), -1)
            cv2.imshow('Capture', clone)

            key = cv2.waitKey(1)
            if key == ord('q') or key == ord('Q'):
                break

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)

if __name__ == "__main__":
    display_camera = DisplayCamera()
    display_camera.run()
