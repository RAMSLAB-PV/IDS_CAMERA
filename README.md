# IDS Camera Manager

`Ids_Camera_Manager.py` is a Python script that manages cameras using the IDS Peak library. It provides methods to configure and control IDS cameras.

## Features

- Initialize and open IDS cameras by serial number.
- Manage camera settings and configurations.
- Capture images and stream video from the camera.

## Requirements

- Python 3.x
- IDS Peak Library
- OpenCV
- JSON
- Threading
- Os
- Sys

## Installation

1. **Install the required Python packages:**
    ```sh
    pip install -r requirements.txt
    ```

2. **Install the IDS Peak SDK:**
    Install [IDS peak 2.11.0 extended](https://en.ids-imaging.com/download-details/1008066.html)  

## Usage

1. **Initialize the Camera Manager:**
   A. **Any Connected Cameras - Single**
     ```python
    from Ids_Camera_Manager import CameraManager

    cam_manager = CameraManager()
    ret = cam_manager.startcamera_auto()    #OTHER OPTIONS .startcamera_manual() .startcamera_load()
    if ret:
        print("status camera ok")
    else:
        print("No camera connected")
    ```
     If more than one is connected, the first one found will be selected.
   
   B. **Specificied "S/N" Cameras - Single**
     ```python
    from Ids_Camera_Manager import CameraManager

    sn = ["S/N"]
    cam_manager = CameraManager(sn)
    ret = cam_manager.startcamera_auto()    #OTHER OPTIONS .startcamera_manual() .startcamera_load()
    if ret:
        print("status camera ok")
    else:
        print("No camera connected")
    ```
     
    C. **Any Connected Cameras - Multi**
     ```python
    from Ids_Camera_Manager import CameraManager
    cameras_manager = []
    while True:
        cam_manager = CameraManager()
        ret = cam_manager.startcamera_auto()    #OTHER OPTIONS .startcamera_manual() .startcamera_load()
        if ret:
            camera_managers.append(cam_manager)
        else:
            break
    ```
   D. **Specificied "S/N" Cameras - Multi**
    ```python
    from Ids_Camera_Manager import CameraManager
    SNS = ["S/N1", "S/N2", ...]
    
    for sn in SNS
        cam_manager = CameraManager(sn)
        ret = cam_manager.startcamera_auto()    #OTHER OPTIONS .startcamera_manual() .startcamera_load()
        if ret:
            camera_managers.append(cam_manager)
        else:
            break
    ```

2. **Capture an image:**
    ```python
    image = camera_manager.get_image()
    
    ```

## Methods

- `__init__(self, camera_id=None)`: Initializes the `CameraManager` class.
- `open_camera(self)`: Opens the camera with the specified ID.
- `get_image(self)`: capture the image of the related camera_manager.
- Additional methods to manage camera settings and configurations.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
