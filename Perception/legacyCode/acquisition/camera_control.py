import gxipy as gx


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
