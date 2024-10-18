import time
import os
import shutil
import cv2
import os
import json
import numpy as np
import random
import torch
import torch.nn as nn
import torch.optim as optim
from torch.autograd import Variable
import torchvision.utils as vutils
from torchvision import transforms, models
import torch.backends.cudnn as cudnn
from matplotlib import pyplot as plt

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def remove_circle(image,coords,r):
    b, a = coords
    h,w = image.shape[:2]
    y,x = np.ogrid[-a:h-a, -b:w-b]
    mask = x*x + y*y <= r*r

    image[mask] = 0

    return image

def draw_circle(image,coords,r):
    b, a = coords
    h,w = image.shape[:2]
    y,x = np.ogrid[-a:h-a, -b:w-b]
    mask = x*x + y*y <= r*r

    image[mask] = 255

    return image

def process_image(img):
    
    h,w = img.shape[:2]
    
    pt=0
    pb=0
    pl=0
    pr=0
    
    if h>w:
        pad_left=int((h-w)/2)
        pl=pad_left
        pad_right=h-w-pad_left
        pr=pad_right
        constant= cv2.copyMakeBorder(img,0,0,pad_left,pad_right,cv2.BORDER_CONSTANT,value=0)
    else:
        pad_top=int((w-h)/2)
        pt=pad_top
        pad_bottom=w-h-pad_top
        pb=pad_bottom
        constant= cv2.copyMakeBorder(img,pad_top,pad_bottom,0,0,cv2.BORDER_CONSTANT,value=0)

    letterBox=constant.copy()

    tensor_size=256

    image = cv2.resize(constant, (tensor_size, tensor_size))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  
    
    image=image.transpose((2, 0, 1))/255

    return image, letterBox


##image,letterbox = process_image('infrared/01_right.png')


def get_corners(img,model):
    image,letterbox = process_image(img)

    h,w = letterbox.shape[:2]



    images=torch.from_numpy(np.array([image])).to(device).float()


    recon_batch = model(images)

    pred = (recon_batch[0].cpu().detach().numpy())

    img=pred.transpose(1,2,0)

    lezero=np.zeros((256,256))

    base_im=np.sum(img,axis=2)
##    cv2.imshow('base_im',base_im)
##    print(base_im.shape)
    p0=list(np.unravel_index(base_im.argmax(), base_im.shape))[::-1]
##    print(p0)
    base_im= remove_circle(base_im,p0,15)
##    cv2.imshow('base_im_1',base_im)
    p1=list(np.unravel_index(base_im.argmax(), base_im.shape))[::-1]
##    print(p1)
    base_im= remove_circle(base_im,p1,15)
    p2=list(np.unravel_index(base_im.argmax(), base_im.shape))[::-1]
##    print(p2)
    base_im= remove_circle(base_im,p2,15)
    p3=list(np.unravel_index(base_im.argmax(), base_im.shape))[::-1]
##    print(p3)


    absolute_points=np.array([p0,p1,p2,p3])
##    print(absolute_points)
##    print(np.mean(absolute_points,axis=0))
    center=np.mean(absolute_points,axis=0)
    relative_points=absolute_points-center
    angles=np.arctan2(relative_points[:,0],relative_points[:,1])
    indices=np.argsort(angles)
    absolute_points=absolute_points[indices]

##    letterbox = cv2.resize(letterbox, (256, 256))


    return absolute_points,letterbox

def get_depth_size(left_im, right_im,model,upper_corner_x,upper_corner_y):

    left_points,lim=get_corners(left_im,model)
    right_points,rim=get_corners(right_im,model)

    cnt=left_points
    cathetus=(cnt[1]-cnt[0])
    cathetus2=(cnt[2]-cnt[1])

    if (np.dot(cathetus2,cathetus2)>np.dot(cathetus,cathetus)):
        cathetus=cathetus2
    
    angle=np.degrees(np.arctan2(cathetus[0], cathetus[1]))

    if angle>=0:
        angle=(90-angle)+0
    else:
        angle=-(angle+90)+0

##    print(lim.shape)
    factor=lim.shape[0]

    corner_left=cv2.polylines(lim, [left_points], True, (255, 0, 0), 2)
    corner_right=cv2.polylines(rim, [right_points], True, (255, 0, 0), 2)

    # cv2.imshow('corner left', corner_left)
    # cv2.imshow('corner right', corner_right)

    avg_depth=0
    for i in range(4):
        disparity=np.abs(left_points[i][0]-right_points[i][0])
        # print(disparity)
        disparity=factor*disparity/256
        depth=448.6156311035156*75/disparity
        avg_depth+=0.25*depth

##        print('raw depth', depth)

##    print('raw depth', avg_depth)

    correction=1.16

    avg_depth*=correction


    b_px=np.linalg.norm(left_points[1]-left_points[0])*factor/256
    h_px=np.linalg.norm(left_points[2]-left_points[1])*factor/256

    px=(upper_corner_x+np.average(right_points[:,0])*factor/256-320)*avg_depth/448.6156311035156
    py=(upper_corner_y+np.average(right_points[:,1])*factor/256-240)*avg_depth/448.6156311035156

    b_mm=avg_depth*b_px/448.6156311035156
    h_mm=avg_depth*h_px/448.6156311035156

##    print('raw depth', avg_depth)

    if b_mm>h_mm:
        b=b_mm
        h=h_mm
    else:
        b=h_mm
        h=b_mm

    return avg_depth,b,h,px,py,angle




