import gxipy as gx
import sys

class Camera:
    def __init__(self, resolution:tuple, serialNumber:str, orientation:str, exposureTime:int):
        self.device_manager = gx.DeviceManager()

        # Basic Camera Settings
        self.ROIx = resolution[0]
        self.ROIy = resolution[1]
        self.serialNumber = serialNumber
        self.orientation = orientation
        self.exposureTime = exposureTime
        
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
        raw_image = self.cam.data_stream[0].get_image()
        # timestamp = raw_image.get_timestamp()
        frame_id = raw_image.get_frame_id()

        if raw_image is None:
            print("Failed to capture image")

        rgb_image = raw_image.convert("RGB")

        numpy_image = rgb_image.get_numpy_array()

        return numpy_image, frame_id