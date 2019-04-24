import cv2
import os
import numpy as np

fps = 10
demo = cv2.imread('0.jpg')
demo = np.min(demo, axis = 2)
size = demo.shape
size = (size[1],size[0])
print('Building Video......')
cap = cv2.VideoWriter("demo_mrcnn.avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps, size)#'M', 'J', 'P', 'G'
for i in range(200):
    img = cv2.imread(str(i)+'.png')
    cap.write(img)
cap.release()
