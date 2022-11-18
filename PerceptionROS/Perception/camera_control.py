import gxipy as gx
import sys
from enum import Enum



# Camera Orientation Table (To be implemented):
# "KE0220040196" -> LEFT
# 
# 



class CameraOrientation(Enum):
    LEFT = 1
    CENTER = 2
    RIGHT = 3

class Camera:
    def __init__(self, resolution:tuple, serialNumber:str, orientation:str):
        self.device_manager = gx.DeviceManager()
        self.ROIx = resolution[0]
        self.ROIy = resolution[1]
        self.serialNumber = serialNumber
        self.orientation = orientation
        
        dev_num, self.dev_info_list = self.device_manager.update_device_list()
        if dev_num == 0:
            print("No Devices Found")
            sys.exit(2)

    def OnClickOpen(self):
        self.cam = self.device_manager.open_device_by_sn(self.serialNumber)

    def OnClickClose(self):
        self.cam.close_device()
    
    def SetSettings(self):
        # Turn BalanceWhiteAuto ON
        self.cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS)

        # Set ExposureTime
        self.cam.ExposureTime.set(100000)

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
        raw_image = self.cam.data_stream[0].get_image()
        timestamp = raw_image.get_timestamp()
        frame_id = raw_image.get_frame_id()

        # TODO: check from which camera i received the image

        if raw_image is None:
            print("Failed to capture image")

        rgb_image = raw_image.convert("RGB")

        numpy_image = rgb_image.get_numpy_array()

        return numpy_image, timestamp, frame_id, self.orientation


# Camera Opening/Closing/Reseting
def closeAllCameras(cameras):
    for i in cameras:
        print(i.close_device())

def openViaSN(device_manager, sn:str):
    camera = device_manager.open_device_by_sn(sn)
    return camera

def openAllCamerasInRandom(dev_num, device_manager, dev_info_list):
    """ 
        Provide the dev_num, device_manager and dev_info_list that
        you get from the device_manager.update_device_list method
        and return a list with all the camera FDs
    """
    camera_list = []

    for i in range(dev_num):
        devSN = dev_info_list[i].get("sn")
        camera = openViaSN(device_manager, devSN)
        camera_list.append(camera)
    return camera_list

def resetCameras(cameras):
    """ Since cameras cannot be powered on at the same time
    without human error, send a reset signal before doing
    anything to sync the timestamp counters
    """
    for i in cameras:
        print(i.DeviceReset.send_command())

# Acquisition Mode Functions
def switchAcquisitionAllCameras(cameras, signal:bool):
    "Desc: if signal == true then start acquisition, if signal == false then stop"
    for i in cameras:
        if signal == True:
            i.stream_on()
        elif signal == False:
            i.stream_off()

def setContAcquisitionMode(cameras, fps:float):
    for i in cameras:
        print(i.AcquisitionMode.set(gx.GxAcquisitionModeEntry.CONTINUOUS))
        print(i.AcquisitionFrameRateMode.set(gx.GxSwitchEntry.ON))
        print(i.AcquisitionFrameRate.set(fps))

def setTriggerMode(cameras):
    for i in cameras:
        i.TriggerMode.set(gx.GxSwitchEntry.ON)
        i.TriggerSource.set(gx.GxTriggerSourceEntry.SOFTWARE)
    

# ROI and Filters Functions
def setCameraROI(camera, offsetY, offsetX, img_width, img_height):
    camera.OffsetY.set(offsetY)
    camera.OffsetX.set(offsetX)
    camera.Width.set(img_width)
    camera.Height.set(img_height)


def importCameraConfig(camera, filename):
    """ camera = camera_fd, filename = local file path
    """
    camera.import_config_file(filename)

def setSettings(camera):
    print(camera.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS))
    print(camera.BalanceWhiteAuto.get())
