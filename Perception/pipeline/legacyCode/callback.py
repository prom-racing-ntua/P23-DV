from helpers import nanoToSecond
from PIL import Image
import gxipy as gx

# Main Callback Function
def capture_callback(raw_image, cnt):
    if raw_image.get_status() == gx.GxFrameStatusList.INCOMPLETE:
        print("Incomplete Frame")
    else:
        print(cnt)
        cnt += 1
        print(f"Current Frame ID: {raw_image.get_frame_id()}, Timestamp: {nanoToSecond(raw_image.get_timestamp())}")
        rgb_image = raw_image.convert("RGB")
        numpy_image = rgb_image.get_numpy_array()
        img = Image.fromarray(numpy_image, 'RGB')
        print(f"Frame ID:{raw_image.get_frame_id()} image processing done")
