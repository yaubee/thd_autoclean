import json
import os

class CameraParameters:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.context = "context"  # This is not serializable

    def to_dict(self):
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy
        }

    @classmethod
    def from_dict(cls, data):
        return cls(data["fx"], data["fy"], data["cx"], data["cy"])

    def __repr__(self):
        return f"CameraParameters(fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy})"

# Save the instance to JSON
def save_instance_to_json(instance, file_path):
    with open(file_path, 'w') as file:
        json.dump(instance.to_dict(), file, indent=4)

# Load the instance from JSON
def load_instance_from_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
        return CameraParameters.from_dict(data)

# Example usage
if __name__ == "__main__":
    camera_parameters = CameraParameters(1.0, 2.0, 3.0, 4.0)
    file_path = 'tools/camera_parameters.json'

    # Ensure the directory exists
    os.makedirs('tools', exist_ok=True)

    # Save the instance to JSON
    save_instance_to_json(camera_parameters, file_path)
    print(f"Instance has been saved to {file_path}")

    # Load the instance from JSON
    loaded_camera_parameters = load_instance_from_json(file_path)
    print(f"Instance has been loaded: {loaded_camera_parameters}")

    # Access the attributes of the loaded instance
    print(f"fx: {loaded_camera_parameters.fx}")
    print(f"fy: {loaded_camera_parameters.fy}")
    print(f"cx: {loaded_camera_parameters.cx}")
    print(f"cy: {loaded_camera_parameters.cy}")
