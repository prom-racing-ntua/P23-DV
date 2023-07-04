import os
import cv2
import torch
import time
import numpy as np
import math
from ament_index_python import get_package_prefix

from .cnn import *

def initYOLOModel(modelpath, conf=0.75, iou=0.45):
    # Local load has way slower performance for some reason...

    # TODO: See if loading locally makes any difference in the tpu inference performance
    yolov5_local_path = os.path.join(get_package_prefix("perception"), "..", "..", "..", "yolov5")
    yolov7_local_path = "/home/prom/YOLO_models/yolov7"

    if "v5" in modelpath:
        # yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', modelpath, force_reload=True)
        yolo_model = torch.hub.load(yolov5_local_path, 'custom', modelpath, source='local', force_reload=True)
    elif "v7" in modelpath:
        yolo_model = torch.hub.load(yolov7_local_path, 'custom', modelpath, source='local', force_reload=True)

    yolo_model.agnostic = True
    yolo_model.conf = conf
    yolo_model.iou = iou
    return yolo_model

def inferenceYOLO(model, img, tpu=True, debug=False):
    baseTime = time.time()
    paddingTime = 0
    if tpu:
        # YOLO in TPU works only for square input i.e. (640,640) so we pad zeros below the actual image
        padded_img = np.zeros((640,640,3),dtype=np.uint8)
        # The original image is (1024,1280) so resizing it to (512,640) is needed. Note: Keypoints require the full image (1024,1280) for maximum resolution. Therefore, the image size cannot be changed in main()
        padded_img[:512] = cv2.resize(img, (640,512))
        
        paddingTime = (time.time() - baseTime)*1000
        print(padded_img.dtype)
        results = model(padded_img)
        # results.save(f"test{time.time()}.jpg")
    else:
        results = model(img)
    # now results are a numpy array containing xmin, ymin, xmax, ymax, confidence, class in each row
    inferenceTime = (time.time() - baseTime)*1000
    return results.pred[0].numpy(), [paddingTime, inferenceTime]

def initKeypoint(small_modelpath, large_modelpath):
    small_model = VGGLikeV3()
    small_model.load_state_dict(torch.load(small_modelpath,map_location=torch.device('cpu')))
    large_model = LargeCNN()
    large_model.load_state_dict(torch.load(large_modelpath,map_location=torch.device('cpu')))
    return small_model, large_model

def cropResizeCones(yolo_results, image, size_cutoff_small, size_cutoff_large, margin):
    img_h = image.shape[0] # camera image height
    img_w = image.shape[1] # camera image width
    
    # small_bounding_boxes contains (xmin,ymin,xmax,ymax,confidence,class) of small cones from the camera image
    # large_bounding_boxes contains (xmin,ymin,xmax,ymax,confidence,class) of large cones from the camera image
    # bounding boxes with less pixels than size_cutoff are ignored 
    # bounding boxes with width > height are also ignored to avoid dropped cones
    small_bounding_boxes = yolo_results[(yolo_results[:,5]<3) * ((yolo_results[:,2]-yolo_results[:,0])*(yolo_results[:,3]-yolo_results[:,1])>size_cutoff_small) * ((yolo_results[:,2]-yolo_results[:,0])<(yolo_results[:,3]-yolo_results[:,1]))]
    large_bounding_boxes = yolo_results[(yolo_results[:,5]==3) * ((yolo_results[:,2]-yolo_results[:,0])*(yolo_results[:,3]-yolo_results[:,1])>size_cutoff_large) * ((yolo_results[:,2]-yolo_results[:,0])<(yolo_results[:,3]-yolo_results[:,1]))]
    small_cones_imgs = []
    large_cones_imgs = []
    
    classes = []
    cropped_img_corners = []
    
    for i in range(len(small_bounding_boxes)):
        # Find corners of cropped images and add a bit of margin. Be careful of margins pushing the corners out of the original image size!
        # The coordinates in small_bounding_boxes refer to a reduced (512,640) image but now that we are working on a (1024,1280) image a multiplication by 2 is required
        xmin = max(2*math.floor(small_bounding_boxes[i,0])-margin, 0)
        xmax = min(2*math.floor(small_bounding_boxes[i,2])+margin, img_w-1)
        ymin = max(2*math.floor(small_bounding_boxes[i,1])-margin, 0)
        ymax = min(2*math.floor(small_bounding_boxes[i,3])+margin, img_h-1)
        
        # Stack cropped images after they are resized to (48,64), which is the expected input size of VggNet
        cone_img = image[ymin:ymax, xmin:xmax]
        cone_img = cv2.resize(cone_img,dsize=(48,64))
        small_cones_imgs.append(cone_img)
        
        # Stack corners of cropped images
        cropped_img_corners.append([xmin, ymin, xmax, ymax])

        # Stack classes of cropped images
        classes.append(small_bounding_boxes[i,5])
    
    for i in range(len(large_bounding_boxes)):
        # Find corners of cropped images and add a bit of margin. Be careful of margins pushing the corners out of the original image size!
        # The coordinates in large_bounding_boxes refer to a reduced (512,640) image but now that we are working on a (1024,1280) image a multiplication by 2 is required
        xmin = max(2*math.floor(large_bounding_boxes[i,0])-margin, 0)
        xmax = min(2*math.floor(large_bounding_boxes[i,2])+margin, img_w-1)
        ymin = max(2*math.floor(large_bounding_boxes[i,1])-margin, 0)
        ymax = min(2*math.floor(large_bounding_boxes[i,3])+margin, img_h-1)
        
        # Stack cropped images after they are resized to (48,64), which is the expected input size of VggNet
        cone_img = image[ymin:ymax, xmin:xmax]
        cone_img = cv2.resize(cone_img,dsize=(48,64))
        large_cones_imgs.append(cone_img)
        
        # Stack corners of cropped images
        cropped_img_corners.append([xmin, ymin, xmax, ymax])
        
        # Stack classes of cropped images
        classes.append(large_bounding_boxes[i,5])

    return small_cones_imgs, large_cones_imgs, classes, cropped_img_corners


def runKeypoints(small_cones_imgs, large_cones_imgs, small_keypoints_model, large_keypoints_model):
    if (len(small_cones_imgs)>0):
        small_cones_imgs_list = []
        
        # Convert images to a Pytorch-friendly format
        for i in range(len(small_cones_imgs)):
            small_cones_imgs_list.append(torch.from_numpy(small_cones_imgs[i].transpose(2,0,1)).unsqueeze(0).float())
        small_cones_imgs_tensor = torch.cat(small_cones_imgs_list, dim=0)
    
        # Inference
        small_predictions = small_keypoints_model(small_cones_imgs_tensor/255.0).cpu().detach().numpy()
        small_predictions = small_predictions.reshape(small_predictions.shape[0], 7, 2).tolist()
    else:
        small_predictions = []
    
    if (len(large_cones_imgs)>0):
        large_cones_imgs_list = []

        # Convert images to a Pytorch-friendly format
        for i in range(len(large_cones_imgs)):
            large_cones_imgs_list.append(torch.from_numpy(large_cones_imgs[i].transpose(2,0,1)).unsqueeze(0).float())
        large_cones_imgs_tensor = torch.cat(large_cones_imgs_list, dim=0)
        
        # Inference
        large_predictions = large_keypoints_model(large_cones_imgs_tensor).cpu().detach().numpy()
        large_predictions = large_predictions.reshape(large_predictions.shape[0], 11, 2).tolist()
    else:
        large_predictions = []
    
    return small_predictions + large_predictions

def yaw_matrix(yaw):
    return np.array([[np.cos(math.pi*yaw/180), -np.sin(math.pi*yaw/180), 0],
                    [np.sin(math.pi*yaw/180), np.cos(math.pi*yaw/180), 0],
                    [0, 0, 1]])

def pitch_matrix(pitch):
    return np.array([[np.cos(math.pi*pitch/180), 0, np.sin(math.pi*pitch/180)],
                    [0, 1, 0],
                    [-np.sin(math.pi*pitch/180), 0, np.cos(math.pi*pitch/180)]])

def rt_converter(camera, pnp_dist):
    # X_COG = -76
    # Y_COG = 0
    # Z_COG = -105
    X_COG = 0
    Y_COG = 0
    Z_COG = 0

    # Takes distance calculated by solvePnP and calculates range,theta from CoG based on the camera used
    if camera == 'left':
        pitch = -12
        yaw = -25

        cog2camera_pitch = pitch_matrix(pitch)
        cog2camera_yaw = yaw_matrix(yaw)
        cog2camera_rotation = cog2camera_pitch @ cog2camera_yaw
        
        x = -109.34 - X_COG     # X_COG = dist(front, cog)
        y = -5.97 - Y_COG       # Y_COG = divergence from centerline
        z = Z_COG
        cog2camera_translation = np.array([[x], [y], [z]])
        camera2cone_translation = np.array([[pnp_dist[2][0]], [pnp_dist[0][0]], [pnp_dist[1][0]]])
        cone_coords = (cog2camera_rotation @ camera2cone_translation) + cog2camera_translation

        x = cone_coords[0]
        y = cone_coords[1]
        r = np.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)

    elif camera == 'right':
        pitch = -12
        yaw = 25

        cog2camera_pitch = pitch_matrix(pitch)
        cog2camera_yaw = yaw_matrix(yaw)
        cog2camera_rotation = cog2camera_pitch @ cog2camera_yaw
        
        x = -109.34 - X_COG     # X_COG = dist(front, cog)
        y = 5.97 - Y_COG       # Y_COG = divergence from centerline
        z = Z_COG
        cog2camera_translation = np.array([[x], [y], [z]])
        camera2cone_translation = np.array([[pnp_dist[2][0]], [pnp_dist[0][0]], [pnp_dist[1][0]]])
        cone_coords = (cog2camera_rotation @ camera2cone_translation) + cog2camera_translation
        
        x = cone_coords[0]
        y = cone_coords[1]
        r = np.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)


def finalCoordinates(camera, classes, cropped_img_corners, predictions, OffsetY):
    rt = []
    for j in range(len(classes)):
        cone_keypoints = []
        
        # Depending on lens used choose an intrinsic camera matrix
        if camera == 'center':
            cameraMatrix = np.array([[2500, 0, 640], [0, 2500, 512], [0, 0, 1]])
            distCoeffs = np.array([[0, 0, 0, 0, 0]])
        elif (camera=='left' or camera=='right'):
            cameraMatrix = np.array([[1250, 0, 640], [0, 1250, 512], [0, 0, 1]])
            distCoeffs = np.array([[0, 0, 0, 0, 0]])          
        
        # Depending on the cone class set real coordiantes of keypoints
        if classes[j] == 3:
            # Outputs the coordinates of keypoints on the full image (not just ROI)
            for k in range(11):
                x = predictions[j][k][0]*(cropped_img_corners[j][2] - cropped_img_corners[j][0])/48 + cropped_img_corners[j][0]
                y = OffsetY + predictions[j][k][1]*(cropped_img_corners[j][3] - cropped_img_corners[j][1])/64 + cropped_img_corners[j][1]

                cone_keypoints.append([x,y]) 
            cone_keypoints_numpy = np.array(cone_keypoints)
            
            real_coords = np.array([[10.5, 4.5, 0],
                        [8.75, 14.35, 0],
                        [7.24, 22.71, 0],
                        [6.04, 30.62 ,0],
                        [4.54, 38.99, 0],
                        [0, 50.5, 0],
                        [-4.54, 38.99, 0],
                        [-6.04, 30.62 ,0],
                        [-7.24, 22.71, 0],
                        [-8.75, 14.35, 0],
                        [-10.5, 4.5, 0]])
        elif classes[j] < 3:
            # Outputs the coordinates of keypoints on the full image (not just ROI)
            for k in range(7):
                x = predictions[j][k][0]*(cropped_img_corners[j][2] - cropped_img_corners[j][0])/48 + cropped_img_corners[j][0]
                y = OffsetY + predictions[j][k][1]*(cropped_img_corners[j][3] - cropped_img_corners[j][1])/64 + cropped_img_corners[j][1]

                cone_keypoints.append([x,y]) 
            cone_keypoints_numpy = np.array(cone_keypoints)
            
            real_coords = np.array([[7.4, 2.7, 0],
                        [5.8, 11.9, 0],
                        [4.3, 20.5, 0],
                        [0, 32.5 ,0],
                        [-4.3, 20.5, 0],
                        [-5.8, 11.9, 0],
                        [-7.4, 2.7, 0]])
        
        # Use solvePnP to get cone position in camera frame and then find range,theta from car CoG
        _, _, translation_vector = cv2.solvePnP(real_coords, cone_keypoints_numpy, cameraMatrix, distCoeffs, cv2.SOLVEPNP_IPPE)
        rt.append(rt_converter(camera, translation_vector))
    return rt
