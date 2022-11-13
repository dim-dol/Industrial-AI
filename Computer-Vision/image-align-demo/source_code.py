'''
Computer Vision

Final Project

2021254018
김원우
'''

import cv2
import numpy as np
import math

img0 = cv2.imread('D:/06/image/IR_07.png', cv2.IMREAD_COLOR)
img0 = cv2.resize(img0, dsize=(384,216), interpolation=cv2.INTER_AREA)

img1 = cv2.imread('D:/06/image/EO_07.png', cv2.IMREAD_COLOR)
img1 = cv2.resize(img1, dsize=(384,216), interpolation=cv2.INTER_AREA)

img_origin = cv2.hconcat([img0,img1])
cv2.imshow('Origin',img_origin)


fast = cv2.FastFeatureDetector_create(170, True, cv2.FAST_FEATURE_DETECTOR_TYPE_9_16)
kp0 = fast.detect(img0)
kp1 = fast.detect(img1)


circle_none_img0 = np.copy(img0)
circle_none_img1 = np.copy(img1)
circle_img0 = np.copy(img0)
circle_img1 = np.copy(img1)
circle_total_img0 = np.copy(img0)
circle_total_img1 = np.copy(img1)

IR_Feature = cv2.KeyPoint_convert(kp0)
EO_Feature = cv2.KeyPoint_convert(kp1)

### Draw Circle on the Feature
for p in cv2.KeyPoint_convert(kp0):
    cv2.circle(circle_total_img0, tuple(p), 2, (255, 0, 0), cv2.FILLED)

for p in cv2.KeyPoint_convert(kp1):
    cv2.circle(circle_total_img1, tuple(p), 2, (0, 255, 0), cv2.FILLED)

Total_Feature = cv2.hconcat([circle_total_img0,circle_total_img1])
cv2.imshow('Total_Feature',Total_Feature)

### Center Region ###
x_max = 288
x_min = 96
y_max = 162
y_min = 54

### find same feature ( replace matching )
(result,irx,iry,eox,eoy) = 10000,0,0,0,0

for x,y in IR_Feature:
    for x2, y2 in EO_Feature:
        if y_min < y < y_max and x_min < x < x_max and y_min < y2 < y_max and x_min < x2 < x_max:
            temp = math.sqrt(pow(x-x2,2)+pow(y-y2,2))
            if 4 < temp < result:
                result = temp
                irx, iry, eox, eoy = x,y,x2,y2

cv2.circle(circle_img0, (irx,iry), 2, (255, 0, 0), cv2.FILLED)
cv2.circle(circle_img1, (eox,eoy), 2, (0, 255, 0), cv2.FILLED)

Same_Feature = cv2.hconcat([circle_img0,circle_img1])
cv2.imshow('Find Same_Feature',Same_Feature)


### Common Feature location
print(irx, eox, iry, eoy)

diffx = int(eox - irx)
diffy = int(eoy - iry)

'''
### total feature img
fusion0 = np.zeros((216+diffy,384+diffx,3), dtype=np.uint8)
fusion1 = np.zeros((216+diffy,384+diffx,3), dtype=np.uint8)
fusion0[0:216,0:384,:] = circle_total_img1
fusion1[diffy:216+diffy,diffx:384+diffx,:] = circle_total_img0
fusion_total_result = cv2.addWeighted(fusion0, 0.6,fusion1,0.4,0)
cv2.imshow('Alignment Image total feature',fusion_total_result)


### common feature img
fusion0 = np.zeros((216+diffy,384+diffx,3), dtype=np.uint8)
fusion1 = np.zeros((216+diffy,384+diffx,3), dtype=np.uint8)
fusion0[0:216,0:384,:] = circle_img1
fusion1[diffy:216+diffy,diffx:384+diffx,:] = circle_img0
fusion_feature_result = cv2.addWeighted(fusion0, 0.6,fusion1,0.4,0)
cv2.imshow('Alignment Image common feature',fusion_feature_result)
'''

### circle none img
fusion0 = np.zeros((216+diffy,384+diffx,3), dtype=np.uint8)
fusion1 = np.zeros((216+diffy,384+diffx,3), dtype=np.uint8)
fusion0[0:216,0:384,:] = circle_none_img1
fusion1[diffy:216+diffy,diffx:384+diffx,:] = circle_none_img0
fusion_result = cv2.addWeighted(fusion0, 0.6,fusion1,0.4,0)
cv2.imshow('Alignment Image',fusion_result)

### origin img
origin = cv2.addWeighted(img0, 0.6,img1,0.4,0)
cv2.imshow('ADD_origin', origin)

if cv2.waitKey(0) == 27:
    cv2.destroyAllWindows()
