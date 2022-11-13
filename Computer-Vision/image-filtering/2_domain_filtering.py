from matplotlib import pyplot as plt
import cv2

origin_img = cv2.imread('D:/TestImage/image_House256rgb.png')
noise = cv2.imread('D:/TestImage/noise2.png')

h = origin_img.shape[0]
w = origin_img.shape[1]

noise_img = cv2.resize(noise,(w,h))

img = cv2.addWeighted(origin_img,0.8,noise_img,0.2,0)


print("diameter 값 : ")
diameter = input()

print("SigmaColor 값 : ")
SigmaColor = input()

print("SigmaSpace 값 : ")
SigmaSpace = input()


img_result = cv2.bilateralFilter(img, d=int(diameter), sigmaColor=int(SigmaColor), sigmaSpace=int(SigmaSpace))


cv2.imshow('origin image',img)
cv2.imshow('filtering image',img_result)


cv2.waitKey()


