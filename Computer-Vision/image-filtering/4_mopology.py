import cv2
import matplotlib.pyplot as plt

image = cv2.imread('D:/TestImage/Lena.png',0)

_, binary = cv2.threshold(image, -1,1,cv2.THRESH_BINARY | cv2.THRESH_OTSU)


print('erosion 횟수 입력')
print(' 0 : 취소')
erosion = input()

print('dilation 횟수 입력')
print(' 0 : 취소')
dilation = input()

print('opening 횟수 입력')
print(' 0 : 취소')
opening = input()

print('closing 횟수 입력')
print(' 0 : 취소')
closing = input()

eroded = cv2.morphologyEx(binary, cv2.MORPH_ERODE,(3,3), iterations=int(erosion))
dilated = cv2.morphologyEx(binary, cv2.MORPH_DILATE,(3,3), iterations=int(dilation))
opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=int(opening))
closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=int(opening))

plt.subplot(221)
plt.axis('off')
plt.title('eroded')
plt.imshow(eroded, cmap='gray')
plt.subplot(222)
plt.axis('off')
plt.title('dilated')
plt.imshow(dilated, cmap='gray')
plt.subplot(223)
plt.axis('off')
plt.title('opened')
plt.imshow(opened, cmap='gray')
plt.subplot(224)
plt.axis('off')
plt.title('closed')
plt.imshow(closed, cmap='gray')
plt.show()

