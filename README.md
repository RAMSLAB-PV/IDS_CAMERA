CameraManager
Overview
CameraManager is a Python class that manages a camera using the IDS Peak library. It allows you to configure and control the camera's settings such as Region of Interest (ROI), frame rate (FPS), gain, and exposure time. The program also supports saving and loading camera settings from JSON files and provides a simple command-line interface for manual configuration.

Dependencies
ids_peak library (IDS Peak SDK)
opencv-python (cv2 for image processing and display)
json (for saving and loading settings)
os (for file operations)
threading (for concurrent image acquisition)
Installation
Ensure you have the IDS Peak SDK installed on your system.

Install the required Python packages:

bash
Copia codice
pip install opencv-python
Usage
Basic Usage
Run the script. The program will start and prompt you to configure the camera settings.

bash
Copia codice
python camera_manager.py
Follow the on-screen prompts to adjust settings such as ROI, FPS, gain, and exposure.

Press 'q' to exit the live feed and save the current images.

Manual Configuration
To manually configure the camera settings:

Choose "Start Camera with Manual Settings" option in the prompt.
Follow the interactive prompts to adjust settings.
You can save the settings for future use.
Auto Configuration
To start the camera with default auto settings:

Choose "Start Camera with Auto Settings" option in the prompt.
Load Settings from File
To load settings from a JSON file:

Ensure the settings file (e.g., <camera_serial_number>.json) is in the camera_settings folder.
Choose "Start Camera and Load Settings from File" option in the prompt.
Stop the Camera
The camera acquisition process will stop when you exit the program or press 'q'. The camera will be properly closed, and any resources will be released.

Code Overview
CameraManager Class
__init__(self, camera_id=None): Initializes the camera manager with optional camera ID.
open_camera(self): Opens the camera specified by ID or the first available camera.
prepare_acquisition(self): Prepares the camera for image acquisition.
set_roi(self, x=0, y=0, width=None, height=None): Sets the Region of Interest (ROI).
set_fps(self, fps=None): Sets the frames per second (FPS).
set_Gain(self, GainMode="Continuous", Gain=None): Sets the gain for the camera.
set_Exposure(self, ExposureMode="Continuous", ExposureTime=None): Sets the exposure time.
alloc_and_announce_buffers(self): Allocates and announces buffers for image acquisition.
start_acquisition(self): Starts the image acquisition process.
runtime_frame(self): Continuously acquires and processes frames from the camera.
manual_settings(self): Allows manual configuration of camera settings.
save_settings(self): Saves current settings to a JSON file.
startcamera_manual(self): Starts the camera with manual settings.
startcamera_load(self): Starts the camera with settings loaded from a JSON file.
startcamera_auto(self): Starts the camera with automatic settings.
stopcamera(self): Stops the camera acquisition process and closes the camera.
get_image(self): Returns the current image acquired from the camera.
_SN(self): Returns the serial number of the camera.
Example
An example of how to use the CameraManager class is provided in the script's __main__ section. It demonstrates creating a CameraManager instance, starting it with manual settings, and displaying the camera feed in a window.

Notes
Ensure that the camera is properly connected and recognized by the IDS Peak SDK.
Adjust the code according to your specific camera model and requirements.
For additional help and configuration options, refer to the IDS Peak SDK documentation.
License
This project is licensed under the MIT License. See the LICENSE file for details.