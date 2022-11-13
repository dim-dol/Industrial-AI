'''
프로그래밍 과제 #2
2021254018 김원우

1. Feature Detection

'''
import cv2
import matplotlib.pyplot as plt
import numpy as np

boat1 = cv2.imread("D:/stitching/boat1.jpg",cv2.IMREAD_COLOR)
budapest1 = cv2.imread("D:/stitching/budapest1.jpg",cv2.IMREAD_COLOR)
newspaper1 = cv2.imread("D:/stitching/newspaper1.jpg",cv2.IMREAD_COLOR)
s1 = cv2.imread("D:/stitching/s1.jpg",cv2.IMREAD_COLOR)

th1 = 150
th2 = 100

# cv2.canny(image, th1, th2)
boat1_edges = cv2.Canny(boat1,150,80)
budapest1_edges = cv2.Canny(budapest1,200,150)
newspaper1_edges = cv2.Canny(newspaper1,150,100)
s1_edges = cv2.Canny(s1,350,300)

# cv2.cornerHarris
def harris_corner(img,block,aperture,k):
    show_img = np.copy(img)
    temp1 = cv2.cornerHarris(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),block,aperture,k)
    temp2 = cv2.dilate(temp1,None)
    show_img[temp2>0.01*temp2.max()] = [0,0,255]
    temp2 = cv2.normalize(temp2,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
    show_img = cv2.cvtColor(temp2,cv2.COLOR_GRAY2BGR)
    return show_img

boat1_harris = harris_corner(boat1,6,3,0.04)
budapest1_harris = harris_corner(budapest1,5,3,0.04)
newspaper1_harris = harris_corner(newspaper1,5,3,0.04)
s1_harris = harris_corner(s1,6,3,0.04)

plt.figure(figsize=(10,8))
plt.subplot(221)
plt.axis('off')
plt.title('boat1_canny')
plt.imshow(boat1_edges, cmap='gray')
plt.subplot(222)
plt.axis('off')
plt.title('budapest1_canny')
plt.imshow(budapest1_edges, cmap='gray')
plt.subplot(223)
plt.axis('off')
plt.title('newspaper1_canny')
plt.imshow(newspaper1_edges, cmap='gray')
plt.subplot(224)
plt.axis('off')
plt.title('s1_canny')
plt.imshow(s1_edges, cmap='gray')
plt.tight_layout()
plt.show()

cv2.waitKey()
cv2.destroyAllWindows()

plt.figure(figsize=(16,10))
plt.subplot(221)
plt.axis('off')
plt.title('boat1_harris')
plt.imshow(boat1_harris)
plt.subplot(222)
plt.axis('off')
plt.title('budapest1_harris')
plt.imshow(budapest1_harris)
plt.subplot(223)
plt.axis('off')
plt.title('newspaper1_harris')
plt.imshow(newspaper1_harris)
plt.subplot(224)
plt.axis('off')
plt.title('s1_harris')
plt.imshow(s1_harris)
plt.tight_layout()
plt.show()

cv2.waitKey()
cv2.destroyAllWindows()