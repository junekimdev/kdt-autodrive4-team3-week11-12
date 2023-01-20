import cv2
import numpy as np


def trffic_light_check(img, detection_msg):
  obj_id, x_min, x_max, y_min, y_max = detection_msg
  if obj_id != 5:
    return detection_msg
  else:
    ### 파란색hsv 정의 ####
    lower_blue = (110,100,100)
    upper_blue = (130, 255, 255)
    ######
    roi_img = img[y_min:y_max, x_min: x_max]
    ## 원본 변환 ##
    new_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    ## hsv -> hsv
    hsv_img = cv2.cvtColor(new_img, cv2.COLOR_BGR2HSV)

    mask  = cv2.inRange(hsv_img, lower_blue, upper_blue)

    mask_img = cv2.bitwise_and(new_img, new_img, mask = mask)

    gray_img = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 100, param1 = 250, param2 = 10, minRadius = 5, maxRadius = 20)
    circles = np.uint16(np.around(circles))
    ### 내접하는 정사각형 범위 ####
    for i in circles[0,:]:
        cx, cy, rect_l = i[0], i[1], int(i[2]*1.414)
    rect_x_min = cx-rect_l//2
    rect_x_max = cx+rect_l//2
    rect_y_min = cy-rect_l//2
    rect_y_max = cy+rect_l//2
    
    # 내접하는 영역 check ###

    if "영역안에서 "=="blue":
      obj_id = -1
    else:
      return detection_msg


