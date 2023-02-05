import numpy as np
import torch

def new_boxes_coordinates(image_size, final_size, box_coordinates, crop=0):
    box_coordinates1 = box_coordinates.copy()
    box_coordinates1 = box_coordinates1 - crop/2
    image_size_crop = image_size - crop
    scale = np.divide(final_size, image_size_crop)
    box_coordinates1 = np.multiply(box_coordinates1, scale)
    return box_coordinates1.astype(int)


def xyxy2xywh(x):   # this funcions converts bounding box format from [x1, y1, x2, y2] -> [x, y, w, h]
    y = torch.zeros(x.shape, device = x.device, dtype = x.dtype)
    y[:, 0] = (x[:, 0] + x[:, 2])/2

    y[:, 1] = (x[:, 1] + x[:, 3])/2

    y[:, 2] = abs(x[:, 2] - x[:, 0])
    y[:, 3] = abs(x[:, 3] - x[:, 1])

    return y

def xywh2xyxy(x):
    y = torch.zeros(x.shape, device=x.device, dtype=x.dtype)
    y[:, 0] = (x[:, 0] - x[:, 2] / 2)
    y[:, 1] = (x[:, 1] - x[:, 3] / 2)
    y[:, 2] = (x[:, 0] + x[:, 2] / 2)
    y[:, 3] = (x[:, 1] + x[:, 3] / 2)
    return y

def calculate_Im_size_after_crop(img, crop_size):
    img_size = img.shape[1:3]
    img_size_after_crop = img_size - crop_size
    return img_size_after_crop


def calculate_scale_factors(img, final_size):
    img_size = img.shape[1:3]
    kx = img_size[0]/final_size[0]  # kx and ky are scale factors that we want to apply to the bounding boxes coordinates
    ky = img_size[1]/final_size[1]
    return kx, ky

        