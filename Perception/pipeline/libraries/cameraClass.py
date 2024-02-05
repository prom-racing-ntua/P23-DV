import gxipy as gx
import sys
import time
import cv2
from collections import deque

# For testing purposes!!!
def initializeCameras(roi, exposureTime):
    device_manager = gx.DeviceManager()

    dev_num, dev_info_list, = device_manager.update_device_list()
    if dev_num == 0:
        print("No Cameras Found")
        sys.exit(0)
    
    cameras = []
    
    for i in range(dev_num):
        devSN = dev_info_list[i].get("sn")
        camera = Camera(roi, devSN, 'random', exposureTime)
        cameras.append(camera)

    for device in cameras:
        device.OnClickOpen()
        device.SetSettings()

    return cameras

class Camera:
    def __init__(self, resolution:tuple, serialNumber:str, orientation:str, exposureTime:int):
        self.device_manager = gx.DeviceManager()

        # Basic Camera Settings
        self.ROIx = resolution[0]
        self.ROIy = resolution[1]
        self.serialNumber = serialNumber
        self.orientation = orientation
        self.exposureTime = exposureTime
        #for testing only
        self.isContinuousAcquisition = False
        
        dev_num, self.dev_info_list = self.device_manager.update_device_list()
        if dev_num == 0:
            print("No Devices Found")
            sys.exit(2)

    def OnClickOpen(self):
        self.cam = self.device_manager.open_device_by_sn(self.serialNumber)

    def OnClickClose(self):
        self.cam.close_device()

    def grab_image(self):
        if not self.isContinuousAcquisition:
            self.TriggerCamera()
        img = self.AcquireImage()
        return img
    
    def visualize_image(self, img, title):
        cv2.imshow(title, img)
        cv2.waitKey(0)

    def grab_and_visualize_image(self, title):
        img = self.grab_image()
        self.visualize_image(img, title)

    def SetAutoExposureSettings(self):
        self.isContinuousAcquisition = True
        self.cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
        self.cam.AcquisitionMode.set(gx.GxAcquisitionModeEntry.CONTINUOUS)
        # self.cam.ExpectedGrayValue.set(130)
        self.cam.ExposureAuto.set(gx.GxAutoEntry.CONTINUOUS)

    def TestAuto(self):
        self.SetAutoExposureSettings()

        exposure_values = deque(maxlen=8)
        for i in range(200):
            exposure_values.append(self.cam.ExposureTime.get())
            if i!=0 and i % 10 == 0:
                percentage_change = ((exposure_values[-1] - exposure_values[0])/exposure_values[0])*100
                # print(str(exposure_values[-1]) + " - " + str(exposure_values[0]))
                # print("Percentage change after " + str(i) + " = " + str(percentage_change))
                if exposure_values[-1] != exposure_values[0] and percentage_change < 15 :
                    break
            # print("Image #" + str(i) + " exposure = " + str(self.cam.ExposureTime.get()))
            self.grab_image()
        
        
        print("exposure set to " + str(self.cam.ExposureTime.get()))
        self.isContinuousAcquisition=False
        self.exposureTime = self.cam.ExposureTime.get()
        self.cam.ExposureAuto.set(gx.GxAutoEntry.OFF)
        self.cam.TriggerMode.set(gx.GxTriggerSourceEntry.SOFTWARE)
        self.cam.TriggerMode.set(gx.GxSwitchEntry.ON)
    
    def TestSettingsChange(self):
        self.cam.stream_off()
        self.cam.ExposureTime.set(45000.0)
        self.cam.stream_on()
        self.cam.TriggerSoftware.send_command()
        raw_image = self.cam.data_stream[0].get_image()
        rgb_image = raw_image.convert("RGB")
        return rgb_image.get_numpy_array()

    
    def SetSettings(self):
        # Turn BalanceWhiteAuto ON
        self.cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS)

        # Set ExposureTime
        self.cam.ExposureAuto.set(gx.GxAutoEntry.OFF)
        self.cam.ExposureTime.set(self.exposureTime)

        # Set TriggerMode to Trigger
        self.cam.TriggerMode.set(gx.GxTriggerSourceEntry.SOFTWARE)
        self.cam.TriggerMode.set(gx.GxSwitchEntry.ON)

        # Set ROI
        self.cam.OffsetY.set(1024-self.ROIy)
        self.cam.OffsetX.set(0)
        self.cam.Width.set(self.ROIx)
        self.cam.Height.set(self.ROIy)   

        # Switch acquisition to ON
        self.cam.stream_on()

    def TriggerCamera(self):
        self.cam.TriggerSoftware.send_command()

    def AcquireImage(self):
        raw_image = self.cam.data_stream[0].get_image(20000)
        # frame_id = raw_image.get_frame_id()

        if raw_image is None:
            print("Failed to capture image")

        rgb_image = raw_image.convert("RGB")

        numpy_image = rgb_image.get_numpy_array()

        return numpy_image