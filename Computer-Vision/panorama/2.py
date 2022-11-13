'''
프로그래밍 과제 #2
2021254018 김원우

2. Matching

'''
import cv2
import numpy as np
import matplotlib.pyplot as plt

img1 = cv2.imread('D:/stitching/s1.jpg', cv2.IMREAD_COLOR)
img2 = cv2.imread('D:/stitching/s2.jpg', cv2.IMREAD_COLOR)

##################################################
surf = cv2.xfeatures2d.SURF_create(10000)
surf.setExtended(True)
surf.setNOctaves(3)
surf.setNOctaveLayers(10)
surf.setUpright(False)

keyPoints, descriptors = surf.detectAndCompute(img1, None)
surf_img1 = cv2.drawKeypoints(img1, keyPoints, None, (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

keyPoints, descriptors = surf.detectAndCompute(img2, None)
surf_img2 = cv2.drawKeypoints(img2, keyPoints, None, (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
##################################################
sift = cv2.xfeatures2d.SIFT_create()

keyPoints = sift.detect(img1,None)
sift_img1 = cv2.drawKeypoints(img1,keyPoints,None, (0,255,0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

keyPoints = sift.detect(img2,None)
sift_img2 = cv2.drawKeypoints(img2,keyPoints,None, (0,255,0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
##################################################
'''
brief = cv2.xfeatures2d.BriefDescriptorExtractor_create(32, True)

keyPoints, descriptors = brief.compute(img, keyPoints)
brief_img = cv2.drawKeypoints(img, keyPoints, None, (0,255,0), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)

keyPoints, descriptors = brief.compute(img2, keyPoints)
brief_img2 = cv2.drawKeypoints(img2, keyPoints, None, (0,255,0), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
#################################################
'''
orb = cv2.ORB_create()
orb.setMaxFeatures(200)

keyPoints = orb.detect(img1, None)
keyPoints, descriptors = orb.compute(img1, keyPoints)
orb_img1 = cv2.drawKeypoints(img1, keyPoints, None, (0, 0, 255), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)

keyPoints = orb.detect(img2, None)
keyPoints, descriptors = orb.compute(img2, keyPoints)
orb_img2 = cv2.drawKeypoints(img2, keyPoints, None, (0, 0, 255), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
###############################################

plt.figure()
plt.subplot(231)
plt.axis('off')
plt.title('SURF')
plt.imshow(surf_img1,aspect='auto')
plt.subplot(232)
plt.axis('off')
plt.title('SIFT')
plt.imshow(sift_img1,aspect='auto')
plt.subplot(233)
plt.axis('off')
plt.title('ORB')
plt.imshow(orb_img1,aspect='auto')
plt.subplot(234)
plt.axis('off')
plt.title('SURF2')
plt.imshow(surf_img2,aspect='auto')
plt.subplot(235)
plt.axis('off')
plt.title('SIFT2')
plt.imshow(sift_img2,aspect='auto')
plt.subplot(236)
plt.axis('off')
plt.title('ORB2')
plt.imshow(orb_img2,aspect='auto')

##############################################

img1 = cv2.imread('D:/stitching/s1.jpg', cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("D:/stitching/s2.jpg", cv2.IMREAD_GRAYSCALE)

detector = cv2.ORB_create(100)
kps0, fea0 = detector.detectAndCompute(img1, None)
kps1, fea1 = detector.detectAndCompute(img2, None)
matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING, False)
matches = matcher.match(fea0,fea1)

pts0 = np.float32([kps0[m.queryIdx].pt for m in matches]).reshape(-1,2)
pts1 = np.float32([kps1[m.queryIdx].pt for m in matches]).reshape(-1,2)
H, mask = cv2.findHomography(pts0, pts1, cv2.RANSAC, 3.0)
print(H)

plt.figure()
plt.subplot(211)
plt.axis('off')
plt.title('all matches')
dbg_img = cv2.drawMatches(img1, kps0, img2, kps1, matches, None)
plt.imshow(dbg_img[:,:,[2,1,0]])
plt.subplot(212)
plt.axis('off')
plt.title('filtered matches')
dbg_img = cv2.drawMatches(img1, kps0, img2, kps1, [m for i,m in enumerate(matches) if mask[i]], None)
plt.imshow(dbg_img[:,:,[2,1,0]])
plt.tight_layout()
plt.show()