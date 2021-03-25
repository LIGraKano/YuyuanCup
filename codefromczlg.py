# Orignal version
# No serial communication version


import numpy as np
import cv2
import time
#得到目标二值图
def get_binary_graph(frame,color):
    color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
                  'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
                  'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
                  }
    goal = color

    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0) #高斯模糊，方便颜色的提取
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)#转换到HSV空间
    erode_hsv = cv2.erode(hsv, None, iterations=2)# 腐蚀 粗的变细***--减少噪点
    inRange_hsv = cv2.inRange(erode_hsv, color_dist[goal]['Lower'], color_dist[goal]['Upper'])#进行二值化
    return inRange_hsv
#得到最小外接正方形
def bounding_rectangle(img):

    cnts = cv2.findContours(img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]#找边界
    if not cnts:
        return np.array([])
    else:
        c = max(cnts,key = cv2.contourArea)#找到最大面积
        rect = cv2.minAreaRect(c)#最小外接矩阵
        box = cv2.boxPoints(rect)#记录矩阵四个点的坐标
        return box
#计算图形几何中心
def calculate_midpoint(box):
    return (box[0,0]+box[1,0]+box[2,0]+box[3,0])/4
#返回于中心的差值（x1-x0）
def get_goal(x1):
    return 300-x1
#主函数
def main():
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    counter=0
    start = time.time()
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("摄像头出问题。")
        else:
            print(frame.shape[0:2])
            img = get_binary_graph(frame,"red")
            rectangle = bounding_rectangle(img)
            if rectangle.size == 0:
                print("NULL")
            else:
                midpoint = calculate_midpoint(rectangle)
                goal = get_goal(midpoint)
                print(goal)
                cv2.drawContours(frame,[np.int0(rectangle)],-1,(0,255,255),2)
        counter = counter + 1
        end = time.time()
        if (end - start >= 1):
            print(counter)
            start = end
            counter = 1
        #frame = cv2.resize(frame, (800, 500))  # 设定窗体尺寸
        #img = cv2.resize(img, (800, 500))
        cv2.imshow('frame', frame)
        #cv2.imshow('img', img)
        k = cv2.waitKey(100) & 0XFF
        if k == 27:
            break
    cv2.destroyAllWindows()
    cap.release()

main()

