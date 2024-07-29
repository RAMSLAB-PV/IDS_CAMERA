import os
import sys
import cv2
import json
import time
import numpy as np
import threading
from ids_peak import ids_peak_ipl_extension as ipl
from ids_peak import ids_peak as peak

class CameraManager:
    """
    CameraManager is a class that manages a camera using the IDS Peak library.
    It provides various methods to configure and control the camera.
    """


    def __init__(self, camera_id=None):
        """
        Initializes the CameraManager class.

        :param camera_id: Serial number of the camera to be managed. If None, the first available camera will be used.
        """
        self.setting_path = "camera_settings"

        self.Msetting = False
        self.m_device = None
        self.m_dataStream = None
        self.m_node_map_remote_device = None
        self.image = None
        self.acquisition_thread = None
        self.running = False
        self.bgr = None
        self.jpg = None
        self.ID = camera_id
        self.print = False
        

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
        try:
            max_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Maximum()
            min_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Minimum()

            if fps is None:
                fps = max_fps

            if 1e6/self.m_node_map_remote_device.FindNode("ExposureTime").Value() < fps:
                self.m_node_map_remote_device.FindNode("ExposureAuto").SetCurrentEntry("Continuous")

            max_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Maximum()
            min_fps = self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Minimum()

            if (fps > max_fps) or (fps < min_fps):
                return False
            self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").SetValue(fps)
            print("FPS of " + self.m_device.SerialNumber() +" set to: " + str(fps))
            
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

                if self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").Value() < 1e3/ExposureTime:
                    self.m_node_map_remote_device.FindNode("AcquisitionFrameRate").SetValue(1e3/ExposureTime)

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
            self.acquisition_thread = threading.Thread(target=self.runtime_frame, daemon=True).start()
            return True
        except Exception as e:
            print(f"Error starting acquisition: {str(e)}")
            return False

    def runtime_frame(self):
        """
        Continuously acquires and processes frames from the camera.
        """
        flag = False
        while self.running:
            buffer = self.m_dataStream.WaitForFinishedBuffer(100000)
            if not buffer.HasImage():
                raise Exception("Buffer does not contain an image.")
            
            self.image = ipl.BufferToImage(buffer).get_numpy_2D()
            self.m_dataStream.QueueBuffer(buffer)
            self.bgr = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
            if self.print:
                cv2.imshow(self.ID, cv2.resize(self.bgr,(self.bgr.shape[1]//2,self.bgr.shape[0]//2)))
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
                        fps = float(value)
                    else:
                        print("invalid FPS")
                        pass

                    if not self.set_fps(fps):
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
            "Exposure": self.m_node_map_remote_device.FindNode("ExposureTime").Value()/1e3
        }

        if not os.path.exists(self.setting_path):
            os.makedirs(self.setting_path)

        file_name = f"{self.ID}.json"
        file_path = os.path.join(self.setting_path, file_name)

        with open(file_path, 'w') as json_file:
            json.dump(settings, json_file, indent=4)

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
    
    def startcamera_load(self, from_manual = False):
        """
        Starts the camera and loads the settings from a JSON file.

        :return: True if the camera is successfully started and configured, False otherwise.
        """
        if not from_manual:
            self.Msetting = True
            if not self.startcamera_auto():
                return False

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


    def get_image(self):
        """
        Returns the current image acquired from the camera.

        :return: The current image in BGR format.
        """
        return self.bgr
    
    def _SN(self):
        """
        Returns the serial number of the camera.

        :return: The serial number of the camera.
        """
        return self.ID

class CameraCalibration:
    def __init__(self, camera):
        self.camera = camera
        self.parameters_path = "camera_parameters"
        self.WR_path = "WorldReference"
        self.cmtx = None
        self.dist = None
        if os.path.exists(os.path.join(self.parameters_path, f"{self.camera._SN()}.json")):
            self.load_calibration()
        self.patternSize = None
        self.squareSize = None
        self.rvec = None
        self.tvec = None
        self.P = None
        if os.path.exists(os.path.join(self.WR_path, f"{self.camera._SN()}.json")):
            self.load_wr()
        
    def mean_corner_difference(corners_new, corners, threshold=50):
        if corners == []:
            return True
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
    
    def calibrate(self, patternSize, squareSize):
        """
        Calibrates the camera using a chessboard pattern.

        :param patternSize: The size of the chessboard pattern (rows, columns).
        :param squareSize: The size of each square in the chessboard pattern.

        :return: True if the camera is successfully calibrated, False otherwise.
        """

        self.patternSize = patternSize
        self.squareSize = squareSize


        chessboard_World = self.cheessboard_pattern_size()

        term_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        #Valid image counter
        valid_images_count = 0
        required_images = 50  # required image for calibration


        while valid_images_count < required_images:
            img = self.camera.get_image()
            ret, corners = cv2.findChessboardCorners(img, patternSize, None)
            if ret and self.mean_corner_difference(corners, imgpoints):
                objpoints.append(chessboard_World)
                corners_refined = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), term_criteria)
                imgpoints.append(corners_refined)

                cv2.drawChessboardCorners(img, patternSize, corners_refined, ret)

                valid_images_count += 1

            cv2.putText(img, f"Valid Images: {valid_images_count}/{required_images}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(self.camera._SN(), cv2.resize(img, (img.shape[1]//2, img.shape[0]//2)))
            cv2.waitKey(1)

        cv2.destroyAllWindows()

        rpj_error, self.cmtx, self.dist, _ , _ = cv2.calibrateCamera(objpoints, imgpoints, img.shape[::-1], None, None)

        if rpj_error < 0.3:
            self.save_calibration()
            return True
        else:
            print("Calibration failed. Reprojection error is too high.")
            return False
        
    def save_calibration(self):
        """
        Saves the camera calibration parameters to a JSON file.

        :param cmtx: The camera matrix.
        :param dist: The distortion coefficients.

        :return: True if the calibration parameters are successfully saved, False otherwise.
        """
        parameters = {
            "CameraMatrix": self.cmtx.tolist(),
            "DistortionCoefficients": self.dist.tolist()
        }

        if not os.path.exists(self.parameters_path):
            os.makedirs(self.parameters_path)

        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.parameters_path, file_name)

        with open(file_path, 'w') as json_file:
            json.dump(parameters, json_file, indent=4)

        return True
    
    def load_calibration(self):
        """
        Loads the camera calibration parameters from a JSON file.

        :return: True if the calibration parameters are successfully loaded, False otherwise.
        """
        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.parameters_path, file_name)

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
    
    def ReferenceFrame_acquisition(self, patternSize, squareSize):
        
        self.patternSize = patternSize
        self.squareSize = squareSize

        last_good = None

        while True:
            img = self.camera.get_image()
            ret, corners = cv2.findChessboardCorners(img, patternSize, None)
            if ret:
                last_good = img.copy()
                cv2.drawChessboardCorners(img, patternSize, corners, ret)

                valid_images_count += 1

            cv2.imshow(self.ID, cv2.resize(img, (img.shape[1]//2, img.shape[0]//2)))
            key = cv2.waitKey(1)
            if key == 32:
                break

        cv2.destroyAllWindows()

        cv2.imwrite(os.path.join(self.WR_path, f"{self.camera._SN()}.png"), last_good)

        return last_good
        
    def get_camera_position(self, wr, patternSize, squareSize):
        
        self.patternSize = patternSize
        self.squareSize = squareSize

        #schacchiera mondo reale
    
        cRW = self.cheessboard_pattern_size() * [1,-1, 1]
        cRW += [self.squareSize, 2*self.squareSize, 0]
            
        # Trova gli angoli della scacchiera
        print('Detecting World Reference')
        ret, corners = cv2.findChessboardCorners(wr, self.patternSize, None)

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
            points_3d = points_3d = np.array([[0, 0, 0],  # Punto 1
                                                [1000, 0, 0],  # Punto 2
                                                [0, 1000, 0],  # Punto 3
                                                [0, 0, 1000]], dtype=np.float32) # Punto 4
            points_2d, _ = cv2.projectPoints(points_3d, self.rvec, self.tvec, self.cmtx, self.dist)

            #img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            wr = cv2.line(wr, np.array(points_2d[0][0],dtype=int), np.array(points_2d[1][0], dtype =int), [255,0,0], 3)
            wr = cv2.line(wr, np.array(points_2d[0][0],dtype=int), np.array(points_2d[2][0], dtype =int), [0,255,0], 3)
            wr = cv2.line(wr, np.array(points_2d[0][0],dtype=int), np.array(points_2d[3][0], dtype =int), [0,0,255], 3)

            cv2.imshow('Chessboard Corners', cv2.resize(wr,(540,960)))
            cv2.waitKey(2000)  # Display each image for a short time
            rotM = cv2.Rodrigues(self.rvec)[0]
            rotM = np.matrix(rotM).T
            
            cameraMATRIX = [0,0,0]
            cameraPosition = rotM*cameraMATRIX.T-rotM*self.tvec
            cameraPosition = cameraPosition.T
            
            cv2.destroyAllWindows()
            
            self.P = np.dot(self.cmtx, np.hstack((cv2.Rodrigues(self.rvec)[0], self.tvec)))

            self.save_wr()


            return cameraPosition
        else:
            print("Angoli della scacchiera non trovati.")
            return None, None, None
    def save_wr(self):
        """
        Saves the World Reference to a JSON file.

        :return: True if the World Reference is successfully saved, False otherwise.
        """
        parameters = {
            "rvec": self.rvec.tolist(),
            "tvec": self.tvec.tolist(),
            "P": self.P.tolist()
        }

        if not os.path.exists(self.WR_path):
            os.makedirs(self.WR_path)

        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.WR_path, file_name)

        with open(file_path, 'w') as json_file:
            json.dump(parameters, json_file, indent=4)
    
    def load_wr(self):
        """
        Loads the World Reference from a JSON file.

        :return: True if the World Reference is successfully loaded, False otherwise.
        """
        file_name = f"{self.camera._SN()}.json"
        file_path = os.path.join(self.WR_path, file_name)

        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return False

        with open(file_path, 'r') as json_file:
            parameters = json.load(json_file)

        try:
            # Convert lists to numpy arrays
            self.rvec = np.array(parameters["rvec"])
            self.tvec = np.array(parameters["tvec"])
            self.P = np.array(parameters["P"])
        except KeyError as e:
            print(f"Missing key in JSON file: {e}")
            return False
        except ValueError as e:
            print(f"Error converting parameters: {e}")
            return False

        return True


#EXAMPLE OF USE
if __name__ == '__main__':

    camera_managers = []
    '''ids = ["4108774181"]
    for id in ids:
        cam_manager = CameraManager(id)
        ret = cam_manager.start()
        if ret:
            camera_managers.append(cam_manager)
        else:
            break'''

    while True:
        cam_manager = CameraManager()
        ret = cam_manager.startcamera_manual()
        if ret:
            camera_managers.append(cam_manager)
        else:
            break

    try:
        while True:
            for cam_manager in camera_managers:
                image = cam_manager.get_image()
                if image is not None:
                    cv2.imshow(cam_manager._SN(), cv2.resize(image,(image.shape[1]//2,image.shape[0]//2)))
            key = cv2.waitKey(1)
            if key == ord('q'):
                for cam_manager in camera_managers:
                    cv2.imwrite(cam_manager._SN() + ".png", cam_manager.get_image())
                break
    finally:
        for cam_manager in camera_managers:
            cam_manager.stopcamera()
        cv2.destroyAllWindows()
        sys.exit(0)