import serial
import time
import random
import model
import step
import birdeye5
import torch
import torchvision
import cv2
import numpy as np
import math

ser = serial.Serial('/dev/ttyAMA3', 9600)
time.sleep(2)
try:
    device = "cpu"
    print('Device:', device)
    darknet = model.DarkNet(conv_only=True, bn=True, init_weight=True)
    mymodel = model.YOLOv1(darknet.features).to(device)
    mymodel.load_state_dict(torch.load("model", map_location=device))
    mymodel.eval()
    toTensor = torchvision.transforms.ToTensor()
    while True:
        if ser.in_waiting > 0:
            park = ser.read()
            park = int.from_bytes(park, byteorder='big')
            if park == 1:
                print(1)
                photos = step.takecamera()
                around_view_transformer = birdeye5.AroundViewTransformer((-2, 2), 0.010417 , (-2, 2), 0.010417)
                birdeye = around_view_transformer.makeAroundView(photos, (384, 384))
                cv2.imwrite("./birdeye.png", birdeye)
                with torch.no_grad():
                    input = toTensor(birdeye)
                    input = input.unsqueeze(0).to(device)
                    outputs = mymodel(input)
                    if outputs.dim() == 3:
                        outputs.unsqeeze(0)
                    pos_x_t = outputs[-1][1]
                    pos_y_t = outputs[-1][0]
                    conf_t = outputs[-1][2]
                    list_points = []
                    for r, confr in enumerate(conf_t):
                        for c, conf in enumerate(confr):
                            if conf.item() > 0.5:
                                y = -(int(64 * r + 64 * pos_y_t[r][c].item()) - 192) / 96
                                x = (int(64 * r + 64 * pos_x_t[r][c].item()) - 192) / 96
                                list_points.append((x, y))
                    print(list_points)
                    if len(list_points) != 4:
                        ser.write(bytes([254,]))
                        continue
                    center_x = (list_points[0][0] + list_points[1][0] + list_points[2][0] + list_points[3][0]) / 4
                    center_y = (list_points[0][1] + list_points[1][1] + list_points[2][1] + list_points[3][1]) / 4
                    min_y = np.min([list_points[0][1], list_points[1][1], list_points[2][1], list_points[3][1]])
                    nowx = 0
                    nowy = 0
                    L = 0.26
                    yaw_angle = np.pi/2
                    v = 0.34
                    target_y = -(center_y - (center_y - min_y))
                    if -0.1 < center_x < 0.1:
                        ser.write(bytes([2, 100]))
                        time.sleep(center_y/v)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
                    elif center_x < -0.1:
                        ser.write(bytes([3, 135]))
                        cx = nowx + L*np.sin(yaw_angle)
                        cy = nowy - L*np.cos(yaw_angle)
                        center_angle = math.atan2(target_y/2 - cy, -center_x/2 - cx)-math.atan2(nowy-cy, nowx-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        
                        yaw_angle = yaw_angle + center_angle
                        if yaw_angle < -np.pi or yaw_angle > np.pi:
                            yaw_angle = (yaw_angle + np.pi)%(2*np.pi) - np.pi
                        ser.write(bytes([3, 45]))
                        cx = -center_x/2 - L*np.sin(yaw_angle)
                        cy = -center_y/2 + L*np.cos(yaw_angle)
                        center_angle = math.atan2(target_y - cy, -center_x - cx)-math.atan2(target_y/2-cy, -center_x/2-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([3, 90]))
                        ser.write(bytes([2, 100]))
                        time.sleep(0.24/0.34)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
                    else:
                        ser.write(bytes([3, 45]))
                        cx = nowx - L*np.sin(yaw_angle)
                        cy = nowy + L*np.cos(yaw_angle)
                        center_angle = math.atan2(target_y/2 - cy, -center_x/2 - cx)-math.atan2(nowy-cy, nowx-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        
                        yaw_angle = yaw_angle + center_angle
                        if yaw_angle < -np.pi or yaw_angle > np.pi:
                            yaw_angle = (yaw_angle + np.pi)%(2*np.pi) - np.pi
                        ser.write(bytes([3, 135]))
                        cx = -center_x/2 + L*np.sin(yaw_angle)
                        cy = -center_y/2 - L*np.cos(yaw_angle)
                        center_angle = math.atan2(target_y - cy, -center_x - cx)-math.atan2(target_y/2-cy, -center_x/2-cx)
                        moving = np.absolute(L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([3, 90]))
                        ser.write(bytes([2, 100]))
                        time.sleep(0.24/0.34)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
        

            elif park == 2:
                print(2)
                photos = step.takecamera()
                around_view_transformer = birdeye5.AroundViewTransformer((-2, 2), 0.010417 , (-2, 2), 0.010417)
                birdeye = around_view_transformer.makeAroundView(photos, (384, 384))
                cv2.imwrite("./birdeye.png", birdeye)
                with torch.no_grad():
                    input = toTensor(birdeye)
                    input = input.unsqueeze(0).to(device)
                    outputs = mymodel(input)
                    if outputs.dim() == 3:
                        outputs.unsqeeze(0)
                    pos_x_t = outputs[-1][1]
                    pos_y_t = outputs[-1][0]
                    conf_t = outputs[-1][2]
                    list_points = []
                    for r, confr in enumerate(conf_t):
                        for c, conf in enumerate(confr):
                            if conf.item() > 0.5:
                                y = -(int(64 * r + 64 * pos_y_t[r][c].item()) - 192) / 96
                                x = (int(64 * r + 64 * pos_x_t[r][c].item()) - 192) / 96
                                list_points.append((x, y))
                    print(list_points)
                    if len(list_points) != 4:
                        ser.write(bytes([254,]))
                        continue
                    center_x = (list_points[0][0] + list_points[1][0] + list_points[2][0] + list_points[3][0]) / 4
                    center_y = (list_points[0][1] + list_points[1][1] + list_points[2][1] + list_points[3][1]) / 4
                    nowx = 0
                    nowy = 0
                    L = 0.26
                    yaw_angle = np.pi/2
                    v = 0.34
                    if center_x < 0:
                        ser.write(bytes([3, 135]))
                        cx = nowx + L*np.sin(yaw_angle)
                        cy = nowy - L*np.cos(yaw_angle)
                        center_angle = math.atan2(-center_y - cy, -center_x - 0.24 - cx)-math.atan2(nowy-cy, nowx-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([3, 45]))
                        ser.write(bytes([1, 100]))
                        time.sleep(0.7)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([3, 90]))
                        ser.write(bytes([2, 100]))
                        time.sleep(1)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
                    if center_x > 0:
                        ser.write(bytes([3, 45]))
                        cx = nowx + L*np.sin(yaw_angle)
                        cy = nowy - L*np.cos(yaw_angle)
                        center_angle = math.atan2(-center_y - cy, -center_x + 0.24 - cx)-math.atan2(nowy-cy, nowx-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([3, 135]))
                        ser.write(bytes([1, 100]))
                        time.sleep(0.7)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([3, 90]))
                        ser.write(bytes([2, 100]))
                        time.sleep(1)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
            elif park == 3:
                print(3)
                photos = step.takecamera()
                around_view_transformer = birdeye5.AroundViewTransformer((-2, 2), 0.010417 , (-2, 2), 0.010417)
                birdeye = around_view_transformer.makeAroundView(photos, (384, 384))
                cv2.imwrite("./birdeye.png", birdeye)
                with torch.no_grad():
                    input = toTensor(birdeye)
                    input = input.unsqueeze(0).to(device)
                    outputs = mymodel(input)
                    if outputs.dim() == 3:
                        outputs.unsqeeze(0)
                    pos_x_t = outputs[-1][1]
                    pos_y_t = outputs[-1][0]
                    conf_t = outputs[-1][2]
                    list_points = []
                    for r, confr in enumerate(conf_t):
                        for c, conf in enumerate(confr):
                            if conf.item() > 0.5:
                                y = -(int(64 * r + 64 * pos_y_t[r][c].item()) - 192) / 96
                                x = (int(64 * r + 64 * pos_x_t[r][c].item()) - 192) / 96
                                list_points.append((x, y))
                    print(list_points)
                    if len(list_points) != 4:
                        ser.write(bytes([254,]))
                        continue
                    center_x = (list_points[0][0] + list_points[1][0] + list_points[2][0] + list_points[3][0]) / 4
                    center_y = (list_points[0][1] + list_points[1][1] + list_points[2][1] + list_points[3][1]) / 4
                    min_y = np.min([list_points[0][1], list_points[1][1], list_points[2][1], list_points[3][1]])
                    L = 0.26
                    yaw_angle = np.pi/2
                    v = 0.34
                    moving = np.absolute((2*min_y-center_y) / v)
                    print(moving)
                    ser.write(bytes([2, 100]))
                    time.sleep(moving)
                    ser.write(bytes([1, 0]))
                    nowx = 0
                    nowy = -(min_y-(center_y-min_y))
                    if center_x <= -0.24:
                        ser.write(bytes([3, 135]))
                        cx = nowx + L*np.sin(yaw_angle)
                        cy = nowy - L*np.cos(yaw_angle)
                        center_angle = math.atan2(-center_y/2 - cy, -center_x/2 - cx)-math.atan2(nowy-cy, nowx-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        
                        yaw_angle = yaw_angle + center_angle
                        if yaw_angle < -np.pi or yaw_angle > np.pi:
                            yaw_angle = (yaw_angle + np.pi)%(2*np.pi) - np.pi
                        ser.write(bytes([3, 45]))
                        cx = -center_x/2 - L*np.sin(yaw_angle)
                        cy = -center_y/2 + L*np.cos(yaw_angle)
                        center_angle = math.atan2(-center_y - cy, -center_x - cx)-math.atan2(-center_y/2-cy, -center_x/2-cx)
                        moving = np.absolute(L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
                    elif center_x >= 0.24:
                        ser.write(bytes([3, 45]))
                        cx = nowx - L*np.sin(yaw_angle)
                        cy = nowy + L*np.cos(yaw_angle)
                        center_angle = math.atan2(-center_y/2 - cy, -center_x/2 - cx)-math.atan2(nowy-cy, nowx-cx)
                        moving = np.absolute(-L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        
                        yaw_angle = yaw_angle + center_angle
                        if yaw_angle < -np.pi or yaw_angle > np.pi:
                            yaw_angle = (yaw_angle + np.pi)%(2*np.pi) - np.pi
                        ser.write(bytes([3, 135]))
                        cx = -center_x/2 + L*np.sin(yaw_angle)
                        cy = -center_y/2 - L*np.cos(yaw_angle)
                        center_angle = math.atan2(-center_y - cy, -center_x - cx)-math.atan2(-center_y/2-cy, -center_x/2-cx)
                        moving = np.absolute(L * center_angle / v)
                        print(moving)
                        ser.write(bytes([2, 100]))
                        time.sleep(moving)
                        ser.write(bytes([1, 0]))
                        ser.write(bytes([255,]))
                    else:
                        print("not in good position for parallel parking")
                        ser.write(bytes([255,]))
            else:
                print("else error")
                print(park)
                continue
            
                
finally:
    ser.close()
