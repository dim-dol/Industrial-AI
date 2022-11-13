
# 2021-10-19
# 
# Mid Term Project
#
# 2021254018
# 김원우

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from threading import Thread
import tkinter as tk

def order_erosion():
    order.append('erosion')

def order_dilation():
    order.append('dilation')

def order_opening():
    order.append('opening')

def order_closing():
    order.append('closing')

def order_pop():
    if order:
        order.pop()

def order_clear():
    order.clear()


def Controler():
    #
    # 1번 쓰레드
    #
    # Make Button control Morphological Filter func ( erosion, dilation, opening, closing )
    # 버튼 클릭 시 order 리스트에 항목 추가

    window = tk.Tk()

    window.title("Morphology func controler")
    window.geometry("400x400+200+200")
    window.resizable(True,True)

    button1 = tk.Button(window, text='erosion', font='Hack',bd=5, fg='red', command = order_erosion)
    button1.pack(side='top', expand=True, fill='both')

    button2 = tk.Button(window, text='dilation', font='Hack',bd=5, fg='green', command = order_dilation)
    button2.pack(side='top', expand=True, fill='both')

    button3 = tk.Button(window, text='opening', font='Hack',bd=5, fg='blue', command = order_opening)
    button3.pack(side='top', expand=True, fill='both')

    button4 = tk.Button(window, text='closing', font='Hack',bd=5, fg='orange', command = order_closing)
    button4.pack(side='top', expand=True, fill='both')

    button5 = tk.Button(window, text='delete', font='Hack',bd=5, fg='black', command = order_pop)
    button5.pack(side='bottom', expand=True,fill='both')

    button6 = tk.Button(window, text='clear', font='Hack',bd=5, fg='purple', command = order_clear)
    button6.pack(side='bottom', expand=True,fill='both')

    window.mainloop()


def MultiImages(dst, src, col, row):
    # 여러가지 영상을 동시에 볼 수 있도록 2X2 frame 구성
    # 각 영상의 size는 288x512
    dst[(col * 288):(col * 288) + 288, (row * 512):(row * 512) + 512] = src[0:288, 0:512]


def MorphologicalFilter(src, order):
    # controler 로부터 order 리스트에 쌓인 항목들을 순서대로 읽으며
    # 각 항목에 맞는 함수를 수행 ( erosion, dilation, opening, closing )
    for i in order:
        if i == 'erosion':
            src = cv2.morphologyEx(src, cv2.MORPH_ERODE, (3,3), iterations=1)
        elif i == 'dilation':
            src = cv2.morphologyEx(src, cv2.MORPH_DILATE, (3,3), iterations=1)
        elif i == 'opening':
            src = cv2.morphologyEx(src, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=1)
        elif i == 'closing':
            src = cv2.morphologyEx(src, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=1)

    return src

def VideoPlayer():
    image_num = 0

    while capture.isOpened():
        run, frame = capture.read(0)
        if not run:
            print("THE END")
            break

        # 1. 원본 / imshow [0,0]
        img_resize = cv2.resize(frame[:,:,0], dsize=(512,288) )

        # 2. 모폴로지 적용 / [0,1]
        _, binary = cv2.threshold(img_resize, 15,80,cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        morphology = MorphologicalFilter(binary, order)

        # 3. adaptiveThreshold 적용 / [1,0]
        adapt_img = cv2.adaptiveThreshold(img_resize, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 6)

        # 4. canny 적용 / [1,1]
        edges = cv2.Canny(img_resize, 30, 100, apertureSize=3)

        # 2X2 FRAME 생성
        output = np.zeros((288*2,512*2), dtype=np.uint8)

        MultiImages(output, img_resize, 0, 0)
        MultiImages(output, morphology, 0, 1)
        MultiImages(output, adapt_img, 1, 0)
        MultiImages(output, edges, 1, 1)
        cv2.imshow('multiView',output)

        # 키보드 입력 이벤트
        # q 입력시 종료 / s 입력시 현재 이미지 저장
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        elif cv2.waitKey(10) & 0xFF == ord('s'):
            print('image saved')
            # 이미지 저장 경로
            cv2.imwrite('./' + str(image_num) + '.jpg', output)
            image_num = image_num + 1

    capture.release()
    cv2.destroyAllWindows()

order = []  # 모폴로지 함수들을 순서대로 나열할 리스트

# 비디오 파일 경로
capture = cv2.VideoCapture('./video.avi')

# 버튼식 GUI , 비디오 플레이어 실행
th1 = Thread(target=Controler)
th2 = Thread(target=VideoPlayer)

th1.start()
th2.start()
th1.join()
th2.join()




