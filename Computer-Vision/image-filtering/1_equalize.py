import cv2
import numpy as np
import matplotlib.pyplot as plt
# 실행 라이브러리 import

img = cv2.imread('D:/TestImage/Lena.png')
img_b, img_g, img_r = cv2.split(img) # 0:Blue, 1:Green, 2:Red
# 이미지 읽기 및 red, green, blue 성분 추출

img_eq = cv2.equalizeHist(img_r)
hist, bins = np.histogram(img_eq, 256, [0, 255])
plt.fill(hist)
plt.xlabel('histogram')
plt.show()
# 이미지 red 성분의 평탄화, 히스토그램 그리기


cv2.imshow('original',img)
cv2.waitKey()

cv2.imshow('histogram_equalization',img_eq)
cv2.waitKey()

result = cv2.merge((img_b, img_g, img_eq))

cv2.imshow('final colored image',result)
cv2.waitKey()


