import os
import sys
import cv2
import json
import time
import threading
import numpy as np
from tqdm import tqdm
from ids_peak import ids_peak as peak
from ids_peak import ids_peak_ipl_extension as ipl

class CameraManager:
    """
    CameraManager is a class that manages a camera using the IDS Peak library.
    It provides various methods to configure and control the camera.
    """


    def __init__(self, camera_id=None, setting_path="camera_settings"):
        """
        Initializes the CameraManager class.

        :param camera_id: Serial number of the camera to be managed. If None, the first available camera will be used.
        """

        self.setting_path = setting_path

        self.Msetting = False
        self.m_device = None
        self.m_dataStream = None
        self.m_node_map_remote_device = None
        self.image = None
        self.acquisition_thread = None
        self.running = False
        self.frame = None
        self.jpg = None
        self.ID = camera_id
        self.print = False
        self.camera_properties = CameraProperties(self)
        self.function = CameraFunction(self)
        self.frame_flag = False
        self.fps = None
        self.colorMode = "BGR"

    def __copy__(self):
        return CameraManager(self.ID, self.setting_path)
    
    def open_camera(self):
        """
        Opens the camera with the specified ID. If no ID is provided, the first available camera is opened.

        :return: True if the camera is successfully opened, False otherwise.
        """
        try:
            device_manager = peak.DeviceManager.Instance()
            device_manager.Update()
            if device_manager.Devices().empty():
                return False

            device_count = device_manager.Devices().size()
            for i in range(device_count):
                if device_manager.Devices()[i].SerialNumber() == self.ID or self.ID is None:
                    if device_manager.Devices()[i].IsOpenable():
                        self.m_device = device_manager.Devices()[i].OpenDevice(peak.DeviceAccessType_Control)
                        self.m_node_map_remote_device = self.m_device.RemoteDevice().NodeMaps()[0]
                        print("Device opened: " + self.m_device.SerialNumber())
                        self.ID = self.m_device.SerialNumber()
                        return True
        except Exception as e:
            print(f"Error opening camera: {str(e)}")
            return False

    def prepare_acquisition(self):
        """
        Prepares the camera for image acquisition.

        :return: True if the preparation is successful, False otherwise.
        """
        try:
            data_streams = self.m_device.DataStreams()
            if data_streams.empty():
                return False

            self.m_dataStream = data_streams[0].OpenDataStream()
            return True
        except Exception as e:
            print(f"Error preparing acquisition: {str(e)}")
            return False

    def set_roi(self, x=0, y=0, width=None, height=None):
        """
        Sets the Region of Interest (ROI) for the camera.

        :param x: The x-coordinate of the top-left corner of the ROI.
        :param y: The y-coordinate of the top-left corner of the ROI.
        :param width: The width of the ROI. If None, the maximum width is used.
        :param height: The height of the ROI. If None, the maximum height is used.
        :return: True if the ROI is successfully set, False otherwise.
        """
        try:
            x_min = self.m_node_map_remote_device.FindNode("OffsetX").Minimum()
            y_min = self.m_node_map_remote_device.FindNode("OffsetY").Minimum()
            w_min = self.m_node_map_remote_device.FindNode("Width").Minimum()
            h_min = self.m_node_map_remote_device.FindNode("Height").Minimum()

            self.m_node_map_remote_device.FindNode("OffsetX").SetValue(x_min)
            self.m_node_map_remote_device.FindNode("OffsetY").SetValue(y_min)
            self.m_node_map_remote_device.FindNode("Width").SetValue(w_min)
            self.m_node_map_remote_device.FindNode("Height").SetValue(h_min)

            x_max = self.m_node_map_remote_device.FindNode("OffsetX").Maximum()
            y_max = self.m_node_map_remote_device.FindNode("OffsetY").Maximum()
            w_max = self.m_node_map_remote_device.FindNode("Width").Maximum()
            h_max = self.m_node_map_remote_device.FindNode("Height").Maximum()

            if width is None or height is None:
                width = w_max
                height = h_max

            if (x < x_min) or (y < y_min) or (x > x_max) or (y > y_max):
                return False
            elif (width < w_min) or (height < h_min) or ((x + width) > w_max) or ((y + height) > h_max):
                return False
            else:
                self.m_node_map_remote_device.FindNode("OffsetX").SetValue(x)
                self.m_node_map_remote_device.FindNode("OffsetY").SetValue(y)
                self.m_node_map_remote_device.FindNode("Width").SetValue(width)
                self.m_node_map_remote_device.FindNode("Height").SetValue(height)
                print("ROI of " + self.m_device.SerialNumber() +" set to: x=" + str(x) + ", y=" + str(y) + ", width=" + str(width) + ", height=" + str(height))
                return True
        except Exception as e:
            print(f"Error setting ROI: {str(e)}")
            return False
        
    def set_offset_x(self, x=0):
        """
        Sets the x-coordinate of the top-left corner of the ROI.

        :param x: The x-coordinate to set.
        :return: True if the offset is successfully set, False otherwise.
        """
        try:
            x_min = self.m_node_map_remote_device.FindNode("OffsetX").Minimum()
            x_max = self.m_node_map_remote_device.FindNode("OffsetX").Maximum()
            
            if x_min <= x <= x_max:
                self.m_node_map_remote_device.FindNode("OffsetX").SetValue(x)
                print(f"OffsetX of {self.m_device.SerialNumber()} set to: x={x}")
                return True
            else:
                return False
        except Exception as e:
            print(f"Error setting OffsetX: {str(e)}")
            return False

    def set_offset_y(self, y=0):
        """
        Sets the y-coordinate of the top-left corner of the ROI.

        :param y: The y-coordinate to set.
        :return: True if the offset is successfully set, False otherwise.
        """
        try:
            y_min = self.m_node_map_remote_device.FindNode("OffsetY").Minimum()
            y_max = self.m_node_map_remote_device.FindNode("OffsetY").Maximum()
            
            if y_min <= y <= y_max:
                self.m_node_map_remote_device.FindNode("OffsetY").SetValue(y)
                print(f"OffsetY of {self.m_device.SerialNumber()} set to: y={y}")
                return True
            else:
                return False
        except Exception as e:
            print(f"Error setting OffsetY: {str(e)}")
            return False

    def set_width(self, width=None):
        """
        Sets the width of the ROI.

        :param width: The width to set. If None, the maximum width is used.
        :return: True if the width is successfully set, False otherwise.
        """
        try:
            w_min = self.m_node_map_remote_device.FindNode("Width").Minimum()
            w_max = self.m_node_map_remote_device.FindNode("Width").Maximum()

            if width is None:
                width = w_max
            
            if w_min <= width <= w_max:
                self.m_node_map_remote_device.FindNode("Width").SetValue(width)
                print(f"Width of {self.m_device.SerialNumber()} set to: width={width}")
                return True
            else:
                return False
        except Exception as e:
            print(f"Error setting Width: {str(e)}")
            return False

    def set_height(self, height=None):
        """
        Sets the height of the ROI.

        :param height: The height to set. If None, the maximum height is used.
        :return: True if the height is successfully set, False otherwise.
        """
        try:
            h_min = self.m_node_map_remote_device.FindNode("Height").Minimum()
            h_max = self.m_node_map_remote_device.FindNode("Height").Maximum()
            if height is None:
                height = h_max
            if h_min <= height <= h_max:
                self.m_node_map_remote_device.FindNode("Height").SetValue(height)
                print(f"Height of {self.m_device.SerialNumber()} set to: height={height}")
                return True
            else:
                return False
        except Exception as e:
            print(f"Error setting Height: {str(e)}")
            return False

    def set_fps(self, fps=None):
        """
        Sets the frames per second (FPS) for the camera.

        :param fps: The FPS to set. If None, the maximum FPS is used.
        :return: True if the FPS is successfully set, False otherwise.
        """
        if fps is not None:
            self.fps = fps

        try:
            max_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Maximum()
            min_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Minimum()

            if self.fps is None:
                self.fps = max_fps

            if 1e6/self.m_node_map_remote_device.FindNode("ExposureTime").Value() < self.fps:
                self.m_node_map_remote_device.FindNode("ExposureAuto").SetCurrentEntry("Continuous")

            max_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Maximum()
            min_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Minimum()

            if (self.fps > max_fps) or (self.fps < min_fps):
                return False
            self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").SetValue(self.fps)
            print("FPS of " + self.m_device.SerialNumber() +" set to: " + str(self.fps))
            
            return True
        except Exception as e:
            print(f"Error setting FPS: {str(e)}")
            return False
    
    def set_Gain(self, GainMode="Continuous", Gain=None):
        """
        Sets the gain for the camera.

        :param GainMode: The gain mode to set ("Continuous" or "Off").
        :param Gain: The gain value to set if GainMode is "Off".
        :return: True if the gain is successfully set, False otherwise.
        """
        try:
            self.m_node_map_remote_device.FindNode("GainAuto").SetCurrentEntry(GainMode)

            if GainMode == "Off":
                max_gain = self.m_node_map_remote_device.FindNode("Gain").Maximum()
                min_gain = self.m_node_map_remote_device.FindNode("Gain").Minimum()
                if (Gain < min_gain) or (Gain > max_gain):
                    return False
                self.m_node_map_remote_device.FindNode("Gain").SetValue(Gain)
                print("Gain of " + self.m_device.SerialNumber() +" set to: " + str(Gain))
            return True
        except Exception as e:
            print(f"Error setting Gain: {str(e)}")
            return False

    def set_Exposure(self, ExposureMode="Continuous", ExposureTime=None):
        """
        Sets the exposure time for the camera.

        :param ExposureMode: The exposure mode to set ("Continuous" or "Off").
        :param ExposureTime: The exposure time to set if ExposureMode is "Off".
        :return: True if the exposure time is successfully set, False otherwise.
        """
        try:
            self.m_node_map_remote_device.FindNode("ExposureAuto").SetCurrentEntry(ExposureMode)

            if ExposureMode == "Off":
                max_exposure = self.m_node_map_remote_device.FindNode("ExposureTime").Maximum()
                min_exposure = self.m_node_map_remote_device.FindNode("ExposureTime").Minimum()

                if ExposureTime > max_exposure and ExposureTime < min_exposure:
                    return False

                self.m_node_map_remote_device.FindNode("ExposureTime").SetValue(1e3*ExposureTime)
                print("Exposure of " + self.m_device.SerialNumber() +" set to: " + str(ExposureTime))
            return True
        except Exception as e:
            print(f"Error setting Exposure: {str(e)}")
            return False

    def alloc_and_announce_buffers(self):
        """
        Allocates and announces the buffers for image acquisition.

        :return: True if the buffers are successfully allocated and announced, False otherwise.
        """
        try:
            if self.m_dataStream:
                self.m_dataStream.Flush(peak.DataStreamFlushMode_DiscardAll)
                for buffer in self.m_dataStream.AnnouncedBuffers():
                    self.m_dataStream.RevokeBuffer(buffer)

                payload_size = self.m_node_map_remote_device.FindNode("PayloadSize").Value()
                num_buffers_min_required = self.m_dataStream.NumBuffersAnnouncedMinRequired()

                for count in range(num_buffers_min_required):
                    buffer = self.m_dataStream.AllocAndAnnounceBuffer(payload_size)
                    self.m_dataStream.QueueBuffer(buffer)

                return True
        except Exception as e:
            print(f"Error allocating and announcing buffers: {str(e)}")
            return False

    def start_acquisition(self):
        """
        Starts the image acquisition process.

        :return: True if the acquisition is successfully started, False otherwise.
        """
        try:
            self.m_dataStream.StartAcquisition(peak.AcquisitionStartMode_Default, peak.DataStream.INFINITE_NUMBER)
            if self.Msetting:
                self.m_node_map_remote_device.FindNode("TLParamsLocked").SetValue(0)
            else:
                self.m_node_map_remote_device.FindNode("TLParamsLocked").SetValue(1)
            self.m_node_map_remote_device.FindNode("AcquisitionStart").Execute()
            self.running = True
            self.acquisition_thread = threading.Thread(target=self.runtime_frame, daemon=False)
            self.acquisition_thread.start()
            return True
        except Exception as e:
            print(f"Error starting acquisition: {str(e)}")
            return False

    def runtime_frame(self):
        """
        Continuously acquires and processes frames from the camera.
        """
        flag = False
        print("Acquisition started for " + self.ID)
        while self.running:
            buffer = self.m_dataStream.WaitForFinishedBuffer(100)

            if not buffer.HasImage():
                raise Exception("Buffer does not contain an image.")
            
            self.image = ipl.BufferToImage(buffer).get_numpy_2D()
            self.frame_flag = True
            self.m_dataStream.QueueBuffer(buffer)
            if self.print:
                self.frame = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
                cv2.imshow(self.ID, cv2.resize(self.frame,(self.frame.shape[1]//2,self.frame.shape[0]//2)))
                cv2.waitKey(1)
                flag = True
            else:
                if flag:
                    cv2.destroyAllWindows()
                    flag = False
                    
        return True
    
    def manual_settings(self):
        """
        Allows the user to manually configure the camera settings through a series of prompts.
        """

        self.m_node_map_remote_device.FindNode("TLParamsLocked").SetValue(0)
        while True:
            self.print = True
            print("Choose a setting to change:")
            print("1. ROI")
            print("2. FPS")
            print("3. Gain")
            print("4. Exposure")
            print("5. Exit")
            setting = input("Setting: ")
            if setting == "1":
                while True:
                    print("Current ROI settings: "+ "X OFFSET: " + str(self.m_node_map_remote_device.FindNode("OffsetX").Value()) + ", Y OFFSET: " + str(self.m_node_map_remote_device.FindNode("OffsetY").Value()) + ", WIDTH: " + str(self.m_node_map_remote_device.FindNode("Width").Value()) + ", HEIGHT: " + str(self.m_node_map_remote_device.FindNode("Height").Value()))
                    print("Choose a setting to change:")
                    print("1. Offset X")
                    print("2. Offset Y")
                    print("3. Width")
                    print("4. Height")
                    print("5. Exit")
                    roi_setting = input("Setting: ")
                    if roi_setting == "1":
                        while True:
                            print("set X OFFSET between " + str(self.m_node_map_remote_device.FindNode("OffsetX").Minimum()) + " and " + str(self.m_node_map_remote_device.FindNode("OffsetX").Maximum()) + ": ")
                            value = input()
                            if value:
                                x = int(value)
                            else:
                                pass

                            if not self.set_offset_x(x):
                                print("Invalid Offset X")
                                pass
                            else:
                                break
                            
                    elif roi_setting == "2":
                        while True:
                            print("set Y OFFSET between " + str(self.m_node_map_remote_device.FindNode("OffsetY").Minimum()) + " and " + str(self.m_node_map_remote_device.FindNode("OffsetY").Maximum()) + ": ")
                            value = input()
                            if value:
                                y = int(value)
                            else:
                                pass

                            if not self.set_offset_y(y):
                                print("Invalid Offset Y")
                                pass
                            else:                                
                                break
                            
                    elif roi_setting == "3":
                        while True:
                            print("set WIDTH between " + str(self.m_node_map_remote_device.FindNode("Width").Minimum()) + " and " + str(self.m_node_map_remote_device.FindNode("Width").Maximum()) + ": ")
                            value = input()
                            if value:
                                width = int(value)
                            else:
                                pass

                            if not self.set_width(width):
                                print("Invalid Width")
                                pass
                            else:
                                break
                            
                    elif roi_setting == "4":
                        while True:
                            print("set HEIGHT between " + str(self.m_node_map_remote_device.FindNode("Height").Minimum()) + " and " + str(self.m_node_map_remote_device.FindNode("Height").Maximum()) + ": ")
                            value = input()
                            if value:
                                height = int(value)
                            else:
                                pass

                            if not self.set_height(height):
                                print("Invalid Height")
                                pass
                            else:
                                break
                            
                    elif roi_setting == "5":
                        break
                    else:
                        print("Invalid setting")
            elif setting == "2":
                while True:
                    print("Current FPS: " + str(self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Value()))
                    print("FPS: ")
                    value = input()
                    if value:
                        self.fps = float(value)
                    else:
                        print("invalid FPS")
                        pass

                    if not self.set_fps(self.fps):
                        print("Invalid FPS")
                        pass
                    else:
                        break
                    
            elif setting == "3":
                while True:
                    print("Current Gain: " + str(self.m_node_map_remote_device.FindNode("Gain").Value()))
                    print("set Gain between: " + str(self.m_node_map_remote_device.FindNode("Gain").Minimum()) + " and " + str(self.m_node_map_remote_device.FindNode("Gain").Maximum())+ ": ")
                    value = input()
                    if value:
                        GainValue = float(value)
                    else:
                        print("invalid Gain")
                        pass

                    if not self.set_Gain(GainMode="Off", Gain=GainValue):
                        print("Invalid Gain Mode")
                        pass
                    else:
                        break
                    
            elif setting == "4":
                while True:
                    print("Current Exposure: " + str(self.m_node_map_remote_device.FindNode("ExposureTime").Value()/1e3))
                    print("set Exposure between: " + str(self.m_node_map_remote_device.FindNode("ExposureTime").Minimum()/1e3) + " ms and " + str(self.m_node_map_remote_device.FindNode("ExposureTime").Maximum()/1e3) + " ms: ")
                    value = input()
                    if value:
                        ExposureValue = float(value)
                    else:
                        print("invalid Exposure")
                        pass

                    if not self.set_Exposure(ExposureMode="Off", ExposureTime=ExposureValue):
                        print("Invalid Exposure Mode")
                        pass
                    else:
                        break
            elif setting == "5":
                self.print = False
                break
            else:
                print("Invalid setting")
        print("Set values for " + str(self.ID) +" : ")
        print("ROI:"+ "X OFFSET: " + str(self.m_node_map_remote_device.FindNode("OffsetX").Value()) + ", Y OFFSET: " + str(self.m_node_map_remote_device.FindNode("OffsetY").Value()) + ", WIDTH: " + str(self.m_node_map_remote_device.FindNode("Width").Value()) + ", HEIGHT: " + str(self.m_node_map_remote_device.FindNode("Height").Value()))
        print("FPS: " + str(self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Value()))
        print("Gain: " + str(self.m_node_map_remote_device.FindNode("Gain").Value()))
        print("Exposure: " + str(self.m_node_map_remote_device.FindNode("ExposureTime").Value()/1e3))
        self.m_node_map_remote_device.FindNode("TLParamsLocked").SetValue(1)
        self.fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Value()

        while True:
            save = input("Do you want to save the settings? (y/n): ")
            if save.upper() == "Y":
                self.save_settings()
                break
            elif save.upper() == "N":
                break
            else:
                pass

        return True
    
    def save_settings(self):
        """
        Saves the current camera settings to a JSON file.

        :return: True if the settings are successfully saved, False otherwise.
        """
        settings = {
            "ROI": {
                "OffsetX": self.m_node_map_remote_device.FindNode("OffsetX").Value(),
                "OffsetY": self.m_node_map_remote_device.FindNode("OffsetY").Value(),
                "Width": self.m_node_map_remote_device.FindNode("Width").Value(),
                "Height": self.m_node_map_remote_device.FindNode("Height").Value()
            },
            "FPS": self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Value(),
            "Gain": self.m_node_map_remote_device.FindNode("Gain").Value(),
            "Exposure": self.m_node_map_remote_device.FindNode("ExposureTime").Value() / 1e3
        }

        if not os.path.exists(self.setting_path):
            os.makedirs(self.setting_path)

        file_name = f"{self.ID}.json"
        file_path = os.path.join(self.setting_path, file_name)

        if os.path.exists(file_path):
            with open(file_path, 'r') as json_file:
                existing_settings = json.load(json_file)
        else:
            existing_settings = {}

        # Update or add the settings
        existing_settings["ROI"] = settings["ROI"]
        existing_settings["FPS"] = settings["FPS"]
        existing_settings["Gain"] = settings["Gain"]
        existing_settings["Exposure"] = settings["Exposure"]

        with open(file_path, 'w') as json_file:
            json.dump(existing_settings, json_file, indent=4)

        return True

    def startcamera_manual(self):
        """
        Starts the camera and allows the user to manually configure the settings.

        :return: True if the camera is successfully started and configured, False otherwise.
        """
        self.Msetting = True
        if not self.startcamera_auto():
            return False
        
        if os.path.exists(os.path.join(self.setting_path, f"{self.ID}.json")):
            if not self.startcamera_load(from_manual = True):
                return False
        self.manual_settings()
        return True
    
    def startcamera_load(self, from_manual = False, setting_path:str=None):
        """
        Starts the camera and loads the settings from a JSON file.

        :return: True if the camera is successfully started and configured, False otherwise.
        """
        if not from_manual:
            self.Msetting = True
            if not self.startcamera_auto():
                return False

        if setting_path is not None:
            self.setting_path = setting_path

        file_name = f"{self.ID}.json"
        file_path = os.path.join(self.setting_path, file_name)
        
        if not os.path.exists(file_path):
            print(f"Settings file {file_name} not found in {self.setting_path}")
            return False

        with open(file_path, 'r') as json_file:
            settings = json.load(json_file)

        self.set_width(settings["ROI"]["Width"])
        self.set_height(settings["ROI"]["Height"])
        self.set_offset_x(settings["ROI"]["OffsetX"])
        self.set_offset_y(settings["ROI"]["OffsetY"])
        self.set_fps(settings["FPS"])
        self.set_Gain(GainMode="Off", Gain=settings["Gain"])
        self.set_Exposure(ExposureMode="Off", ExposureTime=settings["Exposure"])

        print("Loaded settings from file: " + file_name)
        print("ROI: X OFFSET: " + str(settings["ROI"]["OffsetX"]) + ", Y OFFSET: " + str(settings["ROI"]["OffsetY"]) + ", WIDTH: " + str(settings["ROI"]["Width"]) + ", HEIGHT: " + str(settings["ROI"]["Height"]))
        print("FPS: " + str(settings["FPS"]))
        print("Gain: " + str(settings["Gain"]))
        print("Exposure: " + str(settings["Exposure"]))
    
        return True

    def startcamera_auto(self):
        """
        Starts the camera with automatic settings.

        :return: True if the camera is successfully started, False otherwise.
        """
        if not self.print:
            peak.Library.Initialize()
        
        if not self.open_camera():
            return False
        
        if not self.prepare_acquisition():
            sys.exit(-2)
        
        if not self.set_roi():
            sys.exit(-3)
        
        if not self.set_fps():
            sys.exit(-4)
        
        if not self.set_Exposure():
            sys.exit(-5)
        
        if not self.set_Gain():
            sys.exit(-6)
        
        if not self.alloc_and_announce_buffers():
            sys.exit(-7)
        time.sleep(5)
        if not self.start_acquisition():
            sys.exit(-8)
        
        return True
    
    def stopcamera(self):
        """
        Stops the camera acquisition process and closes the camera.
        """
        self.running = False

        if self.acquisition_thread and self.acquisition_thread.is_alive():
            self.acquisition_thread.join()

        if self.m_node_map_remote_device:
            self.m_node_map_remote_device.FindNode("AcquisitionStop").Execute()
        if self.m_dataStream:
            self.m_dataStream.StopAcquisition(peak.AcquisitionStopMode_Default)
        
        time.sleep(1)
        peak.Library.Close()


    def get_image(self, colorMode:str=None):
        """
        Returns the current image acquired from the camera.

        :param colorMode: The colorMode of the image to return (predef. --> "BGR" or "Mono8").

        :return: The current image NOT POOL.
        """
        if colorMode is not None:
            self.colorMode = colorMode

        if self.colorMode == "BGR":
            self.frame = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
            return self.frame
        elif self.colorMode == "Mono8":
            return self.image
        else:
            print("Invalid colorMode")
            return None

        
    
    def wait4frame(self, colorMode:str=None):
        """
        Return a new image when it occur.

        :param colorMode: The colorMode of the image to return (predef. --> "BGR" or "Mono8").

        :return: The poolled image.
        """
        if colorMode is not None:
            self.colorMode = colorMode

        while not self.frame_flag:
            time.sleep(0.001)

        self.frame_flag = False
        return self.get_image()

    
    def _SN(self):
        """
        Returns the serial number of the camera.

        :return: The serial number of the camera.
        """
        return self.ID

class CameraProperties:
    def __init__(self, camera:CameraManager, setting_path="camera_settings"):
        self.camera = camera
        self.setting_path = setting_path
        self.cmtx = None
        self.dist = None
        if os.path.exists(os.path.join(self.setting_path, f"{self.camera._SN()}.json")):
            self.load_calibration()
        self.patternSize = None
        self.squareSize = None
        self.rvec = None
        self.tvec = None
        self.P = None
        if os.path.exists(os.path.join(self.setting_path, f"{self.camera._SN()}.json")):
            self.load_wr()
        
    def mean_corner_difference(self, corners_new, corners, threshold=50):
        for corners_old in corners:
            if np.mean(np.linalg.norm(corners_new - corners_old, axis=2)) < threshold:
                return False

        return True

    def cheessboard_pattern_size(self):
        """
        creates a chessboard from pattern size.

        :param patternSize: The size of the chessboard pattern (rows, columns).
        :param squareSize: The size of each square in the chessboard pattern.

        :return: The chessboard points in RealWorld.

        """

        # Prepare chessboard points (0, 0, 0), (1, 0, 0), (2, 0, 0) ..., (2, 5, 0)
        objp = np.zeros((self.patternSize[0] * self.patternSize[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.patternSize[0], 0:self.patternSize[1]].T.reshape(-1, 2)
        objp *= self.squareSize

        return objp

    def calibrate(self, patternSize:tuple[int,int]=(3,6), squareSize=50.8):
        """
        Calibrates the camera using a chessboard pattern.

        :param patternSize: The size of the chessboard pattern (rows, columns).
        :param squareSize: The size of each square in the chessboard pattern.

        :return: True if the camera is successfully calibrated, False otherwise.
        """
        if self.patternSize is None or self.squareSize is None:
            self.patternSize = patternSize
            self.squareSize = squareSize


        chessboard_World = self.cheessboard_pattern_size()


        # Arrays to store object points and image points from all the images.
        imgpoints = []  # 2d points in image plane.

        #Valid image counter
        valid_images_count = 0

        capturing = False
        frames = []
        tempo = 0
        while True and tempo >= 0:
            
            img = self.camera.image

            if capturing:
                tempo = tempo_end-time.time()
                if tempo <= 30:
                    frames.append(img.copy())
                    img = cv2.flip(img, 1)
                    img = cv2.putText(img, f"Capturing for {tempo:.0f} seconds", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    img = cv2.flip(img, 1)
                    img = cv2.putText(img, f"Capturing in {tempo-30:.0f} seconds", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                img = cv2.flip(img, 1)
                img = cv2.putText(img, "Press Space to start capturing", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow(self.camera._SN(), cv2.resize(img, (img.shape[1]//2, img.shape[0]//2)))
            key = cv2.waitKey(1)

            if key == 32:
                tempo_end = time.time()+35
                capturing = True

        frames = np.array(frames)

        for frame in tqdm(frames, desc="Frame Analysis"):
            ret, corners = cv2.findChessboardCorners(cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR), self.patternSize, None)
            if ret:
                frame = cv2.drawChessboardCorners(frame, self.patternSize, corners, ret)
                imgpoints.append(corners)
                valid_images_count += 1
            cv2.imshow(self.camera._SN(), cv2.resize(frame, (frame.shape[1]//2, frame.shape[0]//2)))
            cv2.waitKey(1)

        print(f"Founded {valid_images_count} valid images")

        imgpoints = np.array(imgpoints).squeeze()
        num_batches = valid_images_count//30
        batches = []
        objpoints = [chessboard_World.copy() for i in range(50)]
        # Creare i batch
        cmtxs = []
        dists = []
        for _ in tqdm(range(4*num_batches), desc="Calibration"):
            # Selezionare 50 indici casuali lungo l'asse 0
            indices = np.random.choice(imgpoints.shape[0], 50, replace=False)
            # Creare il batch selezionando gli indici dal tuo array
            batch = imgpoints[indices]
            batch_list = [batch[i] for i in range(imgpoints[indices].shape[0])]
            
            rpj_error, cmtx_temp, dist_temp, _ , _ = cv2.calibrateCamera(objpoints, batch, img.shape[::-1], None, None)
            cmtxs.append(cmtx_temp)
            dists.append(dist_temp)
        
        # Calcolare la media delle matrici di calibrazione
        self.cmtx = np.mean(cmtxs, axis=0)
        self.dist = np.mean(dists, axis=0)

        num_tests = 50

        # Lista per memorizzare gli errori di riproiezione
        reprojection_errors = []

        for _ in range(num_tests):
            # Selezionare un elemento randomico di `imgpoints` (18x2)
            A = imgpoints[np.random.choice(imgpoints.shape[0])]

            # Assumiamo che `chessboard_World` è già un array di dimensioni (18, 3)
            B = chessboard_World

            # Selezionare 9 indici casuali
            selected_indices = np.random.choice(18, 9, replace=False)
            remaining_indices = [i for i in range(18) if i not in selected_indices]

            # Punti selezionati
            A_selected = A[selected_indices]
            B_selected = B[selected_indices]

            # Punti rimanenti
            A_remaining = A[remaining_indices]
            B_remaining = B[remaining_indices]

            # Calcolare `rvec` e `tvec` usando `cv2.solvePnP`
            success, rvec, tvec = cv2.solvePnP(B_selected, A_selected, self.cmtx, self.dist)

            if success:
                # Proiettare i punti rimanenti usando `cv2.projectPoints`
                projected_points, _ = cv2.projectPoints(B_remaining, rvec, tvec, self.cmtx, self.dist)

                # Calcolare l'errore di riproiezione
                error = np.linalg.norm(A_remaining- projected_points.squeeze(),axis=1)
                reprojection_errors.append(np.mean(error))

        # Calcolare la media degli errori di riproiezione
        mean_reprojection_error = np.mean(reprojection_errors)
        if mean_reprojection_error > 1:
            print(f"Mean reprojection error: {mean_reprojection_error:.2f} pixels")
            print("Calibration failed.")
            key = input("Do you want to try again? (y/n): ")
            if key.upper() == "Y":
                self.calibrate()
        else:
            print(f"Mean reprojection error: {mean_reprojection_error:.2f} pixels")
            print("Calibration successful.")
            self.save_calibration()


    def save_calibration(self):
        """
        Saves the camera calibration parameters to a JSON file.

        :return: True if the calibration parameters are successfully saved, False otherwise.
        """
        parameters = {
            "CameraMatrix": self.cmtx.tolist(),
            "DistortionCoefficients": self.dist.tolist()
        }

        if not os.path.exists(self.setting_path):
            os.makedirs(self.setting_path)

        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.setting_path, file_name)

        if os.path.exists(file_path):
            with open(file_path, 'r') as json_file:
                existing_parameters = json.load(json_file)
        else:
            existing_parameters = {}

        # Update or add the calibration parameters
        existing_parameters["CameraMatrix"] = parameters["CameraMatrix"]
        existing_parameters["DistortionCoefficients"] = parameters["DistortionCoefficients"]

        with open(file_path, 'w') as json_file:
            json.dump(existing_parameters, json_file, indent=4)

        self.camera.save_settings()

        return True
    
    def load_calibration(self, folder_path:str=None):
        """
        Loads the camera calibration parameters from a JSON file.
        "CameraMatrix" and "DistortionCoefficients" are expected keys in the JSON file.
        :return: True if the calibration parameters are successfully loaded, False otherwise.
        """
        if folder_path is not None:
            self.setting_path = folder_path

        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.setting_path, file_name)

        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return False

        with open(file_path, 'r') as json_file:
            parameters = json.load(json_file)

        try:
            # Convert lists to numpy arrays
            self.cmtx = np.array(parameters["CameraMatrix"])
            self.dist = np.array(parameters["DistortionCoefficients"])
        except KeyError as e:
            print(f"Missing key in JSON file: {e}")
            return False
        except ValueError as e:
            print(f"Error converting parameters: {e}")
            return False

        return True
    
    def ReferenceFrame_acquisition(self, patternSize:tuple[int,int]=(3,6), squareSize=50.8):
        """
        Acquires a reference frame using a chessboard pattern and saves the image to a file.

        :param patternSize: The number of internal corners per chessboard row and column (default is (3, 6)).
        :param squareSize: The size of each square on the chessboard in millimeters (default is 50.8).

        :return: The last successfully acquired image with detected chessboard corners, or None if no valid image was found.
        """

        if self.patternSize is None or self.squareSize is None:
            self.patternSize = patternSize
            self.squareSize = squareSize

        self.last_good_reference = None
        self.show = True
        counter = 0
        while self.show:
            self.wr = self.camera.get_image()
            ret, corners = cv2.findChessboardCorners(self.wr, self.patternSize, None)
            if ret:
                counter = 0
                self.last_good_reference = self.wr.copy()
                self.wr = cv2.drawChessboardCorners(self.wr, self.patternSize, corners, ret)
            counter += 1
            if counter > 50:
                counter = 0
                self.last_good_reference = None
        cv2.imwrite(os.path.join(self.setting_path, f"{self.camera._SN()}.png"), self.last_good_reference)
        self.get_camera_position()
        return self.last_good_reference
        
    def get_camera_position(self, image_wr:str=None, patternSize:tuple[int,int]=(3,6), squareSize=50.8, offset_xyz:list[float]=None): 
        
        if self.cmtx is None or self.dist is None:
            if not self.load_calibration():
                return False

        if self.patternSize is None or self.squareSize is None:
            self.patternSize = patternSize
            self.squareSize = squareSize

        if offset_xyz is None:
            offset_xyz = [1, (self.patternSize[1] - 1) / 2, 0] * self.squareSize
        
        if offset_xyz == [1, (patternSize[1] - 1) / 2, 0] * squareSize:
            if offset_xyz[0] != self.squareSize:
                offset_xyz = [1, (self.patternSize[1] - 1) / 2, 0] * self.squareSize

        # Caricamento dell'immagine di riferimento
        if image_wr is None:
            if self.last_good_reference is None:
                reference_image_path = os.path.join(self.setting_path, f"{self.camera._SN()}.png")
                if not os.path.exists(reference_image_path):
                    print("Reference frame not found.")
                    return False
                self.wr = cv2.imread(reference_image_path)
            else:
                self.wr = self.last_good_reference.copy()
        else:
            self.wr = cv2.imread(image_wr)
            if self.wr is None:
                print(f"Error loading image from {image_wr}")
                return False

        #schacchiera mondo reale    
        cRW = self.cheessboard_pattern_size() * [1,-1, 1]
        cRW += offset_xyz
            
        # Trova gli angoli della scacchiera
        print('Detecting World Reference')
        ret, corners = cv2.findChessboardCorners(self.wr, self.patternSize, None)

        if ret:
            # Risolvi il problema PnP per ottenere la posa della fotocamera
            _, self.rvec, self.tvec = cv2.solvePnP(
                cRW,
                corners,
                self.cmtx,
                self.dist, 
                False, 
                cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            print("Rotation Vector (rvec):")
            print(self.rvec)
            print("\nTranslation Vector (tvec):")
            print(self.tvec)
            points_3d = np.array([[0, 0, 0],  # Punto 1
                                 [1000, 0, 0],  # Punto 2
                                 [0, 1000, 0],  # Punto 3
                                 [0, 0, 1000]], dtype=np.float32) # Punto 4
            points_2d, _ = cv2.projectPoints(points_3d, self.rvec, self.tvec, self.cmtx, self.dist)

            #img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            self.wr = cv2.line(self.wr, np.array(points_2d[0][0],dtype=int), np.array(points_2d[1][0], dtype =int), [255,0,0], 3)
            self.wr = cv2.line(self.wr, np.array(points_2d[0][0],dtype=int), np.array(points_2d[2][0], dtype =int), [0,255,0], 3)
            self.wr = cv2.line(self.wr, np.array(points_2d[0][0],dtype=int), np.array(points_2d[3][0], dtype =int), [0,0,255], 3)

            rotM = cv2.Rodrigues(self.rvec)[0]
            rotM = np.matrix(rotM).T
            
            cameraMATRIX = [0,0,0]
            cameraPosition = rotM*cameraMATRIX.T-rotM*self.tvec
            cameraPosition = cameraPosition.T
            
            self.P = np.dot(self.cmtx, np.hstack((cv2.Rodrigues(self.rvec)[0], self.tvec)))

            self.save_wr()


            return True
        else:
            print("Angoli della scacchiera non trovati.")
            return False
        
    def save_wr(self):
        """
        Saves the World Reference to a JSON file.

        :return: True if the World Reference is successfully saved, False otherwise.
        """
        parameters = {
            "RotationVector": self.rvec.tolist(),
            "TranslationVector": self.tvec.tolist(),
            "ProjectionMatrix": self.P.tolist()
        }

        if not os.path.exists(self.setting_path):
             os.makedirs(self.setting_path)

        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.setting_path, file_name)

        if os.path.exists(file_path):
            with open(file_path, 'r') as json_file:
                existing_parameters = json.load(json_file)
        else:
            existing_parameters = {}

        # Update or add the World Reference parameters
        existing_parameters["RotationVector"] = parameters["RotationVector"]
        existing_parameters["TranslationVector"] = parameters["TranslationVector"]
        existing_parameters["ProjectionMatrix"] = parameters["ProjectionMatrix"]

        with open(file_path, 'w') as json_file:
            json.dump(existing_parameters, json_file, indent=4)

        return True
    
    def load_wr(self, folder_path:str=None):
        """
        Loads the World Reference from a JSON file.

        :return: True if the World Reference is successfully loaded, False otherwise.
        """

        if folder_path is not None:
            self.setting_path = folder_path

        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.setting_path, file_name)

        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return False

        with open(file_path, 'r') as json_file:
            parameters = json.load(json_file)

        try:
            # Convert lists to numpy arrays
            self.rvec = np.array(parameters["RotationVector"])
            self.tvec = np.array(parameters["TranslationVector"])
            self.P = np.array(parameters["ProjectionMatrix"])
        except KeyError as e:
            print(f"Missing key in JSON file: {e}")
            return False
        except ValueError as e:
            print(f"Error converting parameters: {e}")
            return False

        return True
    
    def get_K(self):
        if self.cmtx is None:
            if not self.load_calibration():
                return False
        return self.cmtx
    def get_D(self):
        if self.cmtx is None:
            if not self.load_calibration():
                return False
        return self.dist
    
    def get_P(self):
        if self.P is None:
            if not self.load_wr():
                return False
        return self.P
    def get_rvec(self):
        if self.rvec is None:
            if not self.load_wr():
                return False
        return self.rvec
    def get_tvec(self):
        if self.tvec is None:
            if not self.load_wr():
                return False
        return self.tvec
    
    def get_wr(self):
        return self.wr
    
    def stop_reference_acquisition(self):
        self.show = False
        return True

class CameraFunction:
    
    def __init__(self, camera:CameraManager):
        self.camera = camera
        self.VideoRecorder = VideoRecorder(self)

class VideoRecorder:

    def __init__(self, function:CameraFunction):
        self.camera = function.camera
        self.video = None
        self.Thread_video_record = None
        self.stop_record = False
        self.duration = None


    def create(self, base_name:str=None, colormode:str='BGR'):
        """
        Records a video from the camera and saves it to a file.

        :param base_name: The base name of the video file.
        :param colormode: The colormode of the video file (predef-->"BGR" or "MONO8").
        :return: True if the video is successfully recorded, False otherwise.
        """
        if base_name is not None:
            video_name = base_name
        else:
            video_name = self.camera._SN()
        extension = ".avi"
        video_name = base_name + extension if base_name else video_name + extension
        counter = 0

        while os.path.exists(video_name):
            counter += 1
            video_name = f"{base_name}_{counter}{extension}"

        isColor = (colormode == 'BGR')
        self.video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'MJPG'), self.camera.fps, 
                                     (self.camera.wait4frame().shape[1], self.camera.wait4frame().shape[0]), isColor=isColor)

        return True
    
    def start(self, duration:int=None):
        """
        Starts recording a video from the camera.

        :param duration: The duration of the video in seconds.
        :return: True if the video is successfully started, False otherwise.
        """
        if duration is not None:
            self.duration = duration

        if self.video is None:
            print("Video not created.")
            return False

        self.stop_record = False
        self.Thread_video_record = threading.Thread(target=self.record)
        self.Thread_video_record.start()
        
        return True
    
    def record(self):
        """
        Records a video from the camera.

        :return: True if the video is successfully recorded, False otherwise.
        """
        start_time = time.time()
        print("Recording started.")
        counter = 0
        while not self.stop_record:
            self.video.write(self.camera.wait4frame())
            counter += 1
            if time.time() - start_time >= self.duration:
                break
        print(f"Recorded {counter} frames.")
        print("Recording stopped.")
        self.video.release()
        return True

    def stop(self):
        """
        Stops recording a video from the camera.

        :return: True if the video is successfully stopped, False otherwise.
        """
        self.stop_record = True
        if self.Thread_video_record is not None:
            self.Thread_video_record.join()

        return True
    

if __name__ == '__main__':


    camera_managers: list[CameraManager] = []
    '''ids = ["4108774181"]
    for id in ids:
        cam_manager = CameraManager(id)
        ret = cam_manager.start()
        if ret:
            camera_managers.append(cam_manager)
        else:
            break'''
    
    """prova record video"""
    i = 0
    while True:
        cam_manager = CameraManager()

        camera_managers.append(cam_manager.__copy__())
        ret = camera_managers[i].startcamera_load()
        i += 1
        if not ret:
            camera_managers = camera_managers[:-1]
            break
    
    for i in range(len(camera_managers)):
        camera_managers[i].function.VideoRecorder.create()

    for cam in camera_managers:
        cam.function.VideoRecorder.start(10)
    
    time.sleep(10)
    for cam in camera_managers:
        cam.stopcamera()
    
    cv2.destroyAllWindows()
    sys.exit(0)



    #cam_manager = CameraManager()
    #ret = cam_manager.startcamera_manual()
    #cam_manager.function.VideoRecorder.create("prova")
    #cam_manager.function.VideoRecorder.start(10)
    #time.sleep(10)
    #cam_manager.stopcamera()
    #cv2.destroyAllWindows()
    #sys.exit(0)


    """prova calibrazione"""
    #cam = CameraManager()
    #ret = cam.startcamera_load()
    #time.sleep(1)
    #cam.camera_properties.calibrate()
    
    
    """prova varia"""
    #while True:
    #    cam_manager = CameraManager()
    #    ret = cam_manager.startcamera_load()
    #    if ret:
    #        camera_managers.append(cam_manager)
    #    else:
    #        break
    #
    #try:
    #    while True:
    #        for cam_manager in camera_managers:
    #            image = cam_manager.get_image()
    #            if image is not None:
    #                cv2.imshow(cam_manager._SN(), cv2.resize(image,(image.shape[1]//2,image.shape[0]//2)))
    #        key = cv2.waitKey(1)
    #        if key == ord('q'):
    #            for cam_manager in camera_managers:
    #                cv2.imwrite(cam_manager._SN() + ".png", cam_manager.get_image())
    #            break
    #finally:
    #    for cam_manager in camera_managers:
    #        cam_manager.stopcamera()
    #    cv2.destroyAllWindows()
    #    sys.exit(0)