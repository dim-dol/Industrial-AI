from matplotlib import pyplot as plt
import cv2
import numpy as np


origin_img = cv2.imread('D:/TestImage/image_Peppers512rgb.png', 0).astype(np.float32) / 255

fft = cv2.dft(origin_img, flags=cv2.DFT_COMPLEX_OUTPUT)
shifted = np.fft.fftshift(fft, axes=[0,1])


print("반지름 값 d ( 0 < d <= 256 ) : ")
d = input()
mask = np.zeros(fft.shape, np.uint8)
mask = cv2.circle(mask, (256,256), int(d), (255,255,255), -1)


print("high pass(0) / low pass(1)")
pass_filter = input()

if pass_filter == "0":
    mask = 255 - mask
    pass_filter = "HIGH"
elif pass_filter == "1":
    pass_filter = "LOW"

shifted *= mask

b_shifted = np.fft.ifftshift(shifted, axes=[0,1])
b_fft = cv2.idft(fft, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT)
mask_new = np.dstack((mask, np.zeros((origin_img.shape[0], origin_img.shape[1]), dtype=np.uint8)))

plt.figure()
plt.subplot(131)
plt.axis('off')
plt.title('original image')
plt.imshow(origin_img, cmap='gray')

plt.subplot(132)
plt.axis('off')
plt.title('mask')
plt.imshow(mask_new, cmap='gray')

plt.subplot(133)
plt.axis('off')
plt.title('IDFT '+pass_filter+' pass filter result')
plt.imshow(b_fft, cmap='gray')
plt.tight_layout()
plt.show()








