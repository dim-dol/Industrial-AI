'''
프로그래밍 과제 #2
2021254018 김원우

3. Panorama

'''
import cv2

boat = []
boat.append(cv2.imread("D:/stitching/boat1.jpg", cv2.IMREAD_COLOR))
boat.append(cv2.imread("D:/stitching/boat2.jpg", cv2.IMREAD_COLOR))
boat.append(cv2.imread("D:/stitching/boat3.jpg", cv2.IMREAD_COLOR))
boat.append(cv2.imread("D:/stitching/boat4.jpg", cv2.IMREAD_COLOR))
boat.append(cv2.imread("D:/stitching/boat5.jpg", cv2.IMREAD_COLOR))
boat.append(cv2.imread("D:/stitching/boat6.jpg", cv2.IMREAD_COLOR))

stitcher = cv2.createStitcher()
ret, pano_boat = stitcher.stitch(boat)

if ret == cv2.STITCHER_OK:
    pano_boat = cv2.resize(pano_boat, dsize=(0, 0), fx=0.2, fy=0.2)

else:
    print('[boat] error during stitching')

#################################################################################
budapest = []
budapest.append(cv2.imread("D:/stitching/budapest1.jpg", cv2.IMREAD_COLOR))
budapest.append(cv2.imread("D:/stitching/budapest2.jpg", cv2.IMREAD_COLOR))
budapest.append(cv2.imread("D:/stitching/budapest3.jpg", cv2.IMREAD_COLOR))
budapest.append(cv2.imread("D:/stitching/budapest4.jpg", cv2.IMREAD_COLOR))
budapest.append(cv2.imread("D:/stitching/budapest5.jpg", cv2.IMREAD_COLOR))
budapest.append(cv2.imread("D:/stitching/budapest6.jpg", cv2.IMREAD_COLOR))

stitcher = cv2.createStitcher()
ret, pano_budapest = stitcher.stitch(budapest)

if ret == cv2.STITCHER_OK:
    pano_budapest = cv2.resize(pano_budapest, dsize=(0, 0), fx=0.2, fy=0.2)

else:
    print('[budapest] error during stitching')
#################################################################################

newspaper1 = cv2.imread("D:/stitching/newspaper1.jpg", cv2.IMREAD_COLOR)
newspaper2 = cv2.imread("D:/stitching/newspaper2.jpg", cv2.IMREAD_COLOR)
newspaper3 = cv2.imread("D:/stitching/newspaper3.jpg", cv2.IMREAD_COLOR)
newspaper4 = cv2.imread("D:/stitching/newspaper4.jpg", cv2.IMREAD_COLOR)
newspaper1 = cv2.resize(newspaper1, dsize=(2250, 1636), interpolation=cv2.INTER_AREA)
newspaper2 = cv2.resize(newspaper2, dsize=(2250, 1636), interpolation=cv2.INTER_AREA)
newspaper3 = cv2.resize(newspaper3, dsize=(2250, 1636), interpolation=cv2.INTER_AREA)
newspaper4 = cv2.resize(newspaper4, dsize=(2250, 1636), interpolation=cv2.INTER_AREA)

newspaper = []
newspaper.append(newspaper1)
newspaper.append(newspaper2)
newspaper.append(newspaper3)
newspaper.append(newspaper4)

stitcher = cv2.createStitcher()
ret, pano_newspaper = stitcher.stitch(newspaper)

if ret == cv2.STITCHER_OK:
    pano_newspaper = cv2.resize(pano_newspaper, dsize=(0, 0), fx=0.2, fy=0.2)

else:
    print('[newspaper] error during stitching')

#################################################################################
bridge = []
bridge.append(cv2.imread("D:/stitching/s1.jpg", cv2.IMREAD_COLOR))

s2_img = cv2.imread("D:/stitching/s2.jpg", cv2.IMREAD_COLOR)
s2_img = s2_img[:,139::,:]
bridge.append(s2_img)

stitcher = cv2.createStitcher()
ret, pano_bridge = stitcher.stitch(bridge)

if ret == cv2.STITCHER_OK:
    pano_bridge = cv2.resize(pano_bridge, dsize=(0, 0), fx=0.5, fy=0.5)

else:
    print('[bridge] error during stitching')
#################################################################################

cv2.imshow('panorama_boat', pano_boat)
cv2.imshow('panorama_budapest', pano_budapest)
cv2.imshow('panorama_newspaper', pano_newspaper)
cv2.imshow('panorama_bridge', pano_bridge)

cv2.waitKey()
cv2.destroyAllWindows()