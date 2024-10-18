#!/usr/bin/env python3
import socket
from ultralytics import YOLO
import cv2
import depthai as dai
import numpy as np
import torch
import torch.nn as nn
from torchvision import transforms, models
from vision_utils import get_depth_size

torch_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

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

    return image,pt,pl

def convrelu(in_channels, out_channels, kernel, padding):
    return nn.Sequential(
        nn.Conv2d(in_channels, out_channels, kernel, padding=padding),
        nn.ReLU(inplace=True),
    )

def delete_circle(image,coords,r):
    b, a = coords
    h,w = image.shape[:2]
    y,x = np.ogrid[-a:h-a, -b:w-b]
    mask = x*x + y*y <= r*r

    image[mask] = 0

    return image

class ResNetUNet(nn.Module):
    def __init__(self, n_class):
        super().__init__()

        self.base_model = models.resnet18(weights='DEFAULT')
        self.base_layers = list(self.base_model.children())

        self.layer0 = nn.Sequential(*self.base_layers[:3]) # size=(N, 64, x.H/2, x.W/2)
        self.layer0_1x1 = convrelu(64, 64, 1, 0)
        self.layer1 = nn.Sequential(*self.base_layers[3:5]) # size=(N, 64, x.H/4, x.W/4)
        self.layer1_1x1 = convrelu(64, 64, 1, 0)
        self.layer2 = self.base_layers[5]  # size=(N, 128, x.H/8, x.W/8)
        self.layer2_1x1 = convrelu(128, 128, 1, 0)
        self.layer3 = self.base_layers[6]  # size=(N, 256, x.H/16, x.W/16)
        self.layer3_1x1 = convrelu(256, 256, 1, 0)
        self.layer4 = self.base_layers[7]  # size=(N, 512, x.H/32, x.W/32)
        self.layer4_1x1 = convrelu(512, 512, 1, 0)

        self.upsample = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)

        self.conv_up3 = convrelu(256 + 512, 512, 3, 1)
        self.conv_up2 = convrelu(128 + 512, 256, 3, 1)
        self.conv_up1 = convrelu(64 + 256, 256, 3, 1)
        self.conv_up0 = convrelu(64 + 256, 128, 3, 1)

        self.conv_original_size0 = convrelu(3, 64, 3, 1)
        self.conv_original_size1 = convrelu(64, 64, 3, 1)
        self.conv_original_size2 = convrelu(64 + 128, 64, 3, 1)

        self.conv_last = nn.Conv2d(64, n_class, 1)

    def forward(self, input):
        x_original = self.conv_original_size0(input)
        x_original = self.conv_original_size1(x_original)

        layer0 = self.layer0(input)
        layer1 = self.layer1(layer0)
        layer2 = self.layer2(layer1)
        layer3 = self.layer3(layer2)
        layer4 = self.layer4(layer3)

        layer4 = self.layer4_1x1(layer4)
        x = self.upsample(layer4)
        layer3 = self.layer3_1x1(layer3)
        x = torch.cat([x, layer3], dim=1)
        x = self.conv_up3(x)

        x = self.upsample(x)
        layer2 = self.layer2_1x1(layer2)
        x = torch.cat([x, layer2], dim=1)
        x = self.conv_up2(x)

        x = self.upsample(x)
        layer1 = self.layer1_1x1(layer1)
        x = torch.cat([x, layer1], dim=1)
        x = self.conv_up1(x)

        x = self.upsample(x)
        layer0 = self.layer0_1x1(layer0)
        x = torch.cat([x, layer0], dim=1)
        x = self.conv_up0(x)

        x = self.upsample(x)
        x = torch.cat([x, x_original], dim=1)
        x = self.conv_original_size2(x)

        out = self.conv_last(x)

        return out


def get_corners(model, crop,border_x,border_y):
    points=[[0,0],[0,0],[0,0],[0,0]]
    
    
    h,w = crop.shape[:2]
    
    if h>2 and w>2:
    
        image,pt,pl = process_image(crop)    
    
        images=torch.from_numpy(np.array([image])).to(torch_device).float()
    
    
        recon_batch = model(images)
    
        pred = (recon_batch[0].cpu().detach().numpy())
    
        img=pred.transpose(1,2,0)
    
        all_corners=np.sum(img,axis=2)
    
    
    
        p0=list(np.unravel_index(all_corners.argmax(), all_corners.shape))[::-1]
        all_corners=delete_circle(all_corners,p0,25)
    
        p1=list(np.unravel_index(all_corners.argmax(), all_corners.shape))[::-1]
        all_corners=delete_circle(all_corners,p1,25)
    
        p2=list(np.unravel_index(all_corners.argmax(), all_corners.shape))[::-1]
        all_corners=delete_circle(all_corners,p2,25)
    
        p3=list(np.unravel_index(all_corners.argmax(), all_corners.shape))[::-1]
        all_corners=delete_circle(all_corners,p3,25)
        
        points=np.array([p0,p1,p2,p3])
        midpoint=np.mean(points,axis=0)
        relative_points=points-midpoint
        
        angles=np.arctan2(relative_points[:,0],relative_points[:,1])
        indices=np.argsort(angles)
        
        points=points[indices]
        
        if h>w:
            points[:,0]=(points[:,0])*h/w-pl*256/w
        else:
            points[:,1]=(points[:,1])*w/h-pt*256/h
    
    points=(points*np.array([w/256,h/256]))
    points[:,0]+=border_x
    points[:,1]+=border_y
    
    points=points.astype(np.int32).reshape((-1, 1, 2))
    
    return points

model = ResNetUNet(4).to(torch_device)
model.load_state_dict(torch.load('UNet3.mod',map_location=torch.device('cpu')))
model.eval()

detector = YOLO('best.pt') 

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setCamera("right")
xoutRight = pipeline.create(dai.node.XLinkOut)
xoutRight.setStreamName("right")

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setCamera("left")
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutLeft.setStreamName("left")

camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_13_MP)
camRgb.setIspScale(1,8)
xoutRgb = pipeline.create(dai.node.XLinkOut)

controlIn = pipeline.create(dai.node.XLinkIn)
configIn = pipeline.create(dai.node.XLinkIn)
ispOut = pipeline.create(dai.node.XLinkOut)

controlIn.setStreamName('control')
configIn.setStreamName('config')
ispOut.setStreamName('isp')

xoutRgb.setStreamName("rgb")

# Properties
camRgb.setPreviewSize(526, 390)
#camRgb.setPreviewSize(1052, 780)

camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Linking
camRgb.preview.link(xoutRgb.input)
controlIn.out.link(camRgb.inputControl)
configIn.out.link(camRgb.inputConfig)
camRgb.isp.link(ispOut.input)
monoLeft.out.link(xoutLeft.input)
monoRight.out.link(xoutRight.input)



ln=0
box_depth,box_width,box_height=0.0,0.0,0.0


##sizelist=[[20,30,15],[16,19,12],[12,15,12],[39,24,15],[25,16,12],[19,32,6]]
##
##boxSizes=np.array(sizelist)
##
##boxSizes=np.sort(boxSizes,axis=1)
##
##print(boxSizes)





####SOCKET
#a=f

# Connect to device and start pipeline
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:

    controlQueue = device.getInputQueue('control')
    configQueue = device.getInputQueue('config')
    ispQueue = device.getOutputQueue('isp')
    leftQueue = device.getOutputQueue('left')
    rightQueue = device.getOutputQueue('right')
    print('Connected cameras:', device.getConnectedCameraFeatures())
    # Print out usb speed
    print('Usb speed:', device.getUsbSpeed().name)
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Device name
    print('Device name:', device.getDeviceName())

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    expTime = 16000
    sensIso = 800
    print("Setting manual exposure, time: ", expTime, "iso: ", sensIso)
    ctrl = dai.CameraControl()
    ctrl.setManualExposure(expTime, sensIso)
    controlQueue.send(ctrl)

    lensPos = 50
    print("Setting manual focus, lens position: ", lensPos)
    ctrl = dai.CameraControl()
    ctrl.setManualFocus(lensPos)
    controlQueue.send(ctrl)

    ctrl = dai.CameraControl()
    ctrl.setManualWhiteBalance(4000)
    controlQueue.send(ctrl)
    na=10

    warmedUp=False
    warmup=0

    lastX=600
    lastY=-400
    lastAngle=0

    #####SOCKET

    # Connect to device and start pipeline
    msg="no box"
    boxSize=[0.32,0.22,0,15]
    det=0.2

    ClientMultiSocket = socket.socket()
    host ='localhost'
    port = 2004
    print('Waiting for connection response')
    ClientMultiSocket.connect((host, port))
    msg=('vision')
    ClientMultiSocket.send(str.encode(msg))

    #Calibration
    sizes = ClientMultiSocket.recv(1024).decode('utf-8')
    print("tamaños",sizes)
    ClientMultiSocket.send(str.encode("Received sizes"))
    holixxx= ClientMultiSocket.recv(1024).decode('utf-8')
    # print("x", holixxx)
    calibrationArea=int(holixxx)

    ClientMultiSocket.send(str.encode("Received calibration area"))
    holixxx= ClientMultiSocket.recv(1024).decode('utf-8')
    # print("x", holixxx)
    leeway=int(holixxx)
    ClientMultiSocket.send(str.encode("Received leeway"))

    sizeArray=sizes.split(";")
    sizeList=[]
    for s in sizeArray:
        sizeN=np.array([int(i) for i in s.split(',')])
        sizeN[0]-=1
        sizeN[1]-=1
        sizeList.append(sizeN)
    boxSizes=np.array(sizeList)
    boxSizes[:,:2]=boxSizes[:,:2]+leeway

    boxSizes=np.sort(boxSizes,axis=1)

    print(boxSizes)


    while True:
        # ctrl = dai.CameraControl()
        # ctrl.setManualExposure(expTime, sensIso)
        # controlQueue.send(ctrl)
##        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

        t_h= 0.2
        t_b= 0.3
        t_d=0.15
        z=0.55

        ispFrames = ispQueue.tryGetAll()
        leftFrames = leftQueue.tryGetAll()
        rightFrames = rightQueue.tryGetAll()

        for leftFrame in leftFrames:
            imageLeft=leftFrame.getCvFrame()
            gray_left=cv2.cvtColor(imageLeft,cv2.COLOR_GRAY2RGB)
        for rightFrame in rightFrames:
            imageRight=rightFrame.getCvFrame()
            gray_right=cv2.cvtColor(imageRight,cv2.COLOR_GRAY2RGB)
        
        for ispFrame in ispFrames:
            image0=ispFrame.getCvFrame()
            imageColor=image0.copy()
            
        ready=False

        msg='no box'
        if 'image0' in locals() and not warmedUp:
            warmup=warmup+1
        if warmup>=15 and not warmedUp:
            warmedUp=True

        if warmedUp:
            res = ClientMultiSocket.recv(1024)
    ##        print('Servidor dice: ', res.decode('utf-8'))
            if res.decode('utf-8') == "Ready":
                ready=True
            elif res.decode('utf-8') == "Busy":
                ready=False    

        
        if 'image0' in locals():
            results = detector(image0.copy(), save=False, conf=0.05, verbose=False)            

            bboxes=np.array(results[0].cpu().numpy().boxes.xyxy)
            
            if len(bboxes)>0:
                centers=np.array(results[0].cpu().numpy().boxes.xywh)
                centers=centers[:,:2]
                center_dif=np.sum(np.abs(centers-np.array([263,195])),axis=1)
                center_args=np.argsort(center_dif)
                bbox=bboxes[center_args[0]]
                bbox=[int(box) for box in bbox]
                window_h=30
                window_w=50
                if bbox[0]>window_w and bbox[1]>window_h and bbox[2]<(526-window_w) and bbox[3]<(390-window_h):
                    pad=8
                    crop=image0[bbox[1]-pad:bbox[3]+pad,bbox[0]-pad:bbox[2]+pad]
                    if 'imageLeft' in locals():
                        if 'imageRight' in locals():
                            crop_right=gray_right[int(0.9*bbox[1])-pad:int(1.32*bbox[3])+pad,int(0.9*bbox[0])-pad:int(1.32*bbox[2])+pad]
                            crop_left=gray_left[int(0.9*bbox[1])-pad:int(1.32*bbox[3])+pad,int(0.9*bbox[0])-pad:int(1.32*bbox[2])+pad]
                            depth,box_width,box_height,px,py,angle=get_depth_size(crop_left, crop_right,model,int(0.9*bbox[0])-pad,int(0.9*bbox[1])-pad)
                            box_depth=755-depth
                            # print('Depth is',box_depth,depth)
                            

                            box_width*=0.1
                            box_height*=0.1
                            box_depth*=0.1

                            
                            original_indices=np.array([0,1,2])
                            
                            size=np.array([box_width,box_height,box_depth])
                            # print('size',size)
                            sorted_indices=np.argsort(size)
                            sorted_size=size[sorted_indices]
                            original_indices=original_indices[sorted_indices]
                            original_indices=np.argsort(original_indices)
                            # print(original_indices)
                            # print(sorted_size)
                            size_differences=np.abs(boxSizes-sorted_size)/boxSizes
##                            print(size_differences)
                            size_differences=np.sum(np.abs(boxSizes-sorted_size),axis=1)
##                            print(size_differences)
                            size_index=np.argmin(size_differences)
                            final_size=boxSizes[size_index]

                            box_width,box_height,box_depth=final_size[original_indices]

                            # print(sorted_size[original_indices],px,py)
                            # print(sorted_size[original_indices],602-py,px-216)

                            reported_x=602-py
                            reported_y=px-216

                            lastX=0.35*lastX+0.65*reported_x
                            lastY=0.35*lastY+0.65*reported_y
                            lastAngle=0.5*lastAngle+0.5*angle

                            if (np.abs(reported_x-lastX)+ np.abs(reported_x-lastX))>15:
                                det=0.2
                            else:
                                det=0.001
                            
                            angle_msg=lastAngle

                            msg=str((reported_x+16)/1000)+','+str((reported_y+6)/1000)+',0,'+str(box_width)+','+str(box_height)+','+str(box_depth)+','+str(angle_msg-0.5)+','+str(det)

                            
                    corners=get_corners(model, crop,bbox[0]-pad,bbox[1]-pad)
                    im = cv2.polylines(image0, [corners], True, (255, 0, 0), 2)
                    im = cv2.putText(
                      img = im,
                      text = ("{:.1f}".format(box_width) +' x '+"{:.1f}".format(box_height) 
                              + ' x ' + "{:.1f}".format(box_depth) + " {:.1f}".format(lastAngle)),
                      org = (100, 100),
                      fontFace = cv2.FONT_HERSHEY_DUPLEX,
                      fontScale = 0.7,
                      color = (125, 246, 55),
                      thickness = 3
                    )

                else:
                    im=image0
            else:
                im=image0

            if warmedUp:
                ClientMultiSocket.send(str.encode(msg))

        
            cv2.imshow('Detection', cv2.resize(im,(1052, 780)))
        

        # Retrieve 'bgr' (opencv format) frame
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
cv2.destroyAllWindows()
