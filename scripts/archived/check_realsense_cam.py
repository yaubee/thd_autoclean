import pyrealsense2 as rs

# Create a context object
context = rs.context()

# List all connected devices
devices = context.query_devices()
for device in devices:
    print("Device found: ", device.get_info(rs.camera_info.name))
    print("Serial number: ", device.get_info(rs.camera_info.serial_number))

desired_serial_number = '218622275676' #d405

device_found = False
for device in devices:
    if device.get_info(rs.camera_info.serial_number) == desired_serial_number:
        device_found = True
        break

if not device_found:
    print(f"Device with serial number {desired_serial_number} not found.")
    exit(1)




