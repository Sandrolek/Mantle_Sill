#!/usr/bin/env python
# -*- coding: utf-8 -*-

# подключение либ
import rospy
import numpy as np
import time

import cv2 as cv
from cv_bridge import CvBridge

from clover import srv
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool
from clover.srv import SetLEDEffect
import math

from std_msgs.msg import Float32MultiArray

from pyzbar import pyzbar

bridge = CvBridge()

# инициализация ноды и сервисов
rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

set_effect(r=0, b=0, g=0)

# константы
TIME_RATE = 0.1
TIME_STABLE = 3
TIME_UP = 3
TIME_DOWN = 3
TIME_WAIT = 1

WIDTH = 320
HEIGHT = 240

FLY_HEIGHT = 1.3

polygon_x = 3.2
polygon_y = 2.4

# создание публикаторов
debug = rospy.Publisher('debug', Image, queue_size=1)
thresh_pub = rospy.Publisher('thresh', Image, queue_size=1)
detect = rospy.Publisher('/Detect', Image, queue_size=1)
image_pub = rospy.Publisher('qr_debug', Image, queue_size=1)
qr_pub = rospy.Publisher('qr_data', String, queue_size=1)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

GAZEBO = False

# маски для стрелки
if GAZEBO:
    masks = {
        'red': ((169, 100, 155), (179, 255, 255), (0, 100, 155), (15, 255, 255)),
        'yellow': ((20, 80, 120), (40, 255, 255)),
        'blue': ((110, 70, 115), (130, 255, 255))
    }
else:
    masks = {
        'red': (np.array([0, 39, 88]), np.array([19, 255, 255]), np.array([169, 44, 88]), np.array([179, 255, 255])),
        'yellow': (np.array([25, 65, 93]), np.array([56, 255, 255])),
        'blue': (np.array([88, 65, 88]), np.array([134, 255, 255]))
    }


relatives = {
        0: 'Down',
        1: 'Left',
        2: 'Up',
        3: 'Right'
    }

cx = 0
cy = 0

f = open("Record.txt", 'w')

# калбэк на тпоик из telem.py
def telem_rate(msg):

    f.write(str(round(msg.data[0], 2)) + '\t' + str(round(msg.data[1], 2)) + '\t' + str(round(msg.data[2], 2)) + '\n')

# калбэк на изображение с камеры, нужен только для теста.
def thresh_cb(data):
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    ret, thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY_INV)

    thresh_pub.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))

# функция полета в точку
def navigate_wait(x=0, y=0, z=0, yaw=math.radians(90), speed=0.3, frame_id='', auto_arm=False, tolerance=0.15):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(TIME_RATE)

# функция расчета расстояния от центра QR кода до центра картинки
def count_dist(x ,y):
    return math.sqrt(abs(x - WIDTH/2)**2 + abs(y - HEIGHT/2)**2)

def image_callback(data):
    global state_qr

    # захват изображения
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    # получение всех кодов на фрейме
    barcodes = pyzbar.decode(frame)
    
    # если нет кодов - дальше делать нечего, уходим
    if len(barcodes) == 0:
        return
    
    # находим код, который ближе всего к центру, ведь если на одном кадре несколько, нас интересует самый близкий к центру (при условии что коптер нормально отцентрировался к заданной точке)
    min_dist = count_dist(barcodes[0].rect[0], barcodes[0].rect[1])
    min_dist_qr = 0
    state_qr = True
    
    for i in range(1, len(barcodes)):
        dist = count_dist(barcodes[i].rect[0], barcodes[i].rect[1])
        if dist < min_dist:
            min_dist = dist
            min_dist_qr = i

    # работаем с выбранным выше кодом, обрисовываем его и публикуем в топик, читаем значение и публикуем его в топик
    barcode = barcodes[min_dist_qr]
    b_data = barcode.data.encode("utf-8")
    b_type = barcode.type
    (x, y, w, h) = barcode.rect
    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 105, 180), 3)
    cv.putText(frame, b_data, (x,y), cv.FONT_HERSHEY_PLAIN, 2, (0, 0, 255))
    xc = x + w/2
    yc = y + h/2
    print ("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))

    qr_pub.publish(b_data)
    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    detect.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

# получение сообщения из qr кода
def catch_qr():

    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
    rospy.sleep(TIME_WAIT)

    global state_qr
    state_qr = False

    time_start = rospy.Time.now()
    time_max = 5

    while not state_qr:
        if rospy.Time.now() - time_start > rospy.Duration(secs = time_max):
            print("Time limit OVER, no QR code found")
            rospy.sleep(TIME_WAIT)
            return "exit"
        rospy.sleep(TIME_RATE)

    qr_data = rospy.wait_for_message('qr_data', String).data

    print("Data is:", qr_data)

    image_sub.unregister()

    return qr_data

# определение направления стрелки (+ цвета)
def find_arrow(data):
    print("finding Arrow")
    
    samples = np.loadtxt('generalsamples.data', np.float32)
    responses = np.loadtxt('generalresponses.data', np.float32)
    responses = responses.reshape((responses.size, 1))

    model = cv.ml.KNearest_create()
    model.train(samples, cv.ml.ROW_SAMPLE, responses)

    #print("model made")

    ############################# testing part #########################

    im = bridge.imgmsg_to_cv2(data, 'bgr8')#[90:160, 120:200]
    out = im.copy()

    hsv = cv.cvtColor(im, cv.COLOR_BGR2HSV)

    kernal = np.ones((5, 5), np.uint8)

    is_color = True
    arrow_color = 0
	
    # идет проверка по всем цветам кроме черного
    for mask in masks:
        if len(masks[mask]) == 4:
            #print("Red mask")
            now_mask = cv.inRange(hsv, masks[mask][0], masks[mask][1]) + cv.inRange(hsv, masks[mask][2], masks[mask][3])
        else:
            #print("NOT red mask")
            now_mask = cv.inRange(hsv, masks[mask][0], masks[mask][1])

        print("Check color", mask)
        debug.publish(bridge.cv2_to_imgmsg(now_mask, 'mono8'))
        rospy.sleep(2)

        contours, hierarchy = cv.findContours(now_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]      

        contours = [i for i in contours if cv.contourArea(i) > 80]

        if len(contours) == 0:
            is_color = False
            continue
    
        is_color = True
        arrow_color = mask
        break

    if not is_color:
        print("Color NOT found")
        arrow_color = 'black'
        gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gray, 70, 255, cv.THRESH_BINARY_INV)
    else:
        print("Color:", arrow_color)
        thresh = now_mask

    # на этом шаге нам уже не важно какого цвета стрелка, мы просто работает с чернобелым изображением
    debug.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))

    contours, hierarchy = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[-2:]

    cnt = max(contours, key=cv.contourArea)

    num = 0
    arr = []

    for cnt in contours:
        [x, y, w, h] = cv.boundingRect(cnt)
        try:
            #print("is contour")
            cv.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
            roi = thresh[y:y + h, x:x + w]
            roismall = cv.resize(roi, (10, 10))
            roismall = roismall.reshape((1, 100))
            roismall = np.float32(roismall)
            retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
            num = int(results[0][0]) # ?
            arr.append((cnt, dists[0][0], num))
            #cv.putText(out, result, (x + w // 2, y + h // 2), 0, 1, (0, 255, 0))
        except cv.Error as e:
            print('Invalid')

    need_arr = min(arr, key=lambda x: x[1])

    num = need_arr[2]

    cv.drawContours(out, [need_arr[0]], -1, (255, 105, 180), 3)

    detect.publish(bridge.cv2_to_imgmsg(out, 'bgr8'))

    print("Arrow is: " + relatives[num])

    d = dict()
    d['dir'] = num
    d['color'] = arrow_color

    return d

def getCenterOfContour_callback(data):
    global cx
    global cy
    im = CvBridge().imgmsg_to_cv2(data, 'bgr8')
    out = im.copy()
    hsv = cv.cvtColor(im, cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsv, (50, 55, 50), (85, 255, 255))

    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]

    if len(contours) != 0:
        cnt = max(contours, key=cv.contourArea)
        [x, y, w, h] = cv.boundingRect(cnt)  # getting the the coordinates of the bigest contour 
        cx = x + w // 2
        cy = y + h // 2

    out = cv.arrowedLine(out, (320 // 2, 240 // 2), (cx, cy), (0, 0, 255), 4)
    out_mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
    vis = np.concatenate((out, out_mask), axis=1)

    debug.publish(CvBridge().cv2_to_imgmsg(vis, 'bgr8'))

def normalize(x, y):
    return x / math.sqrt(x ** 2 + y ** 2), y / math.sqrt(x ** 2 + y ** 2)

# никак не поменявшиеся с прошлых дней функции посадки
def preciseLanding(need_z, indic):  
    global cx
    global cy
    z = need_z
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, getCenterOfContour_callback, queue_size=1)
    rospy.sleep(1)

    x0, y0 = 320 // 2, 240 // 2

    while math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2) > 5:

        # if math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2) < 50:
        #     z -= 0.2
        # if math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2) < 20:
        #     z -= 0.15

        telem = get_telemetry(frame_id='aruco_map')

        dx, dy = normalize(cx - x0, cy - y0)  # get final motion vector 
        dx /= 15  # limit the speed
        dy /= 15
        dy = -dy  # the y-axis of the frame is directed in the opposite direction of the y-axis of the marker map
        # z -= 0.03
        set_position(x=telem.x + dx, y=telem.y + dy, z=z, yaw=math.radians(90), frame_id='aruco_map')
        rospy.sleep(0.1)

    image_sub.unregister()

    if indic == 'black':
        set_effect(effect='fade', r=255, g=255, b=255)
    elif indic == 'red':
        set_effect(effect='fade', r=255, g=0, b=0)
    elif indic == 'blue':
        set_effect(effect='fade', r=0, g=0, b=255)
    elif indic == 'yellow':
        set_effect(effect='fade', r=255, g=255, b=0)

    land()
    rospy.sleep(1)
    arming(False)
    cx, cy = 0, 0

# старт проги
telem_sub = rospy.Subscriber('/telem', Float32MultiArray, telem_rate, queue_size=1)
thresh_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, thresh_cb, queue_size=1)
print("Started")
start_time = time.time()
print("start time:", start_time)
navigate(x=0, y=0, z=FLY_HEIGHT, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(TIME_UP)
print("Got off")

navigate_wait(x=0, y=0, z=FLY_HEIGHT, frame_id='aruco_map')
print("Got to Started point")

navigate_wait(x=0.4, y=0.8, z=1, frame_id='aruco_map')
print("Got to start scan point")

# qr
s = catch_qr()

print(s)

# обработка строки из qr кода
s_obstucle = ""

for i in range(len(s)):
    if s[i] == '\n':
        s_obstucle = s[:i+1]
        break

print(type(s_obstucle))

arr_obstucle = list(map(float, s_obstucle.split()))

for i in range(0, len(arr_obstucle)-1, 2):
    print("Column area x=" + str(arr_obstucle[i]) + ", y=" + str(arr_obstucle[i+1]))

arrow_x = 0
arrow_y = 0

for i in range(len(s_obstucle), len(s)):

    if s[i] == '\n':
        arrow_x, arrow_y = list(map(float, s[len(s_obstucle):i+1].split()))
        break

print("Navigation area x=" + str(arrow_x) + " y=" + str(arrow_y))
print("X:", arrow_x, "Y:", arrow_y)

num_order = s[len(s)-1]

print("Order number:", num_order)

# полет к стрелке
navigate_wait(x=arrow_x, y=arrow_y, z=1.2, frame_id='aruco_map', speed=1)
print("Got to arrow point")

rospy.sleep(4)

arrow = find_arrow(rospy.wait_for_message('main_camera/image_raw', Image))
direction = arrow['dir']
arrow_color = arrow['color']

sector = ''

# выбор сектора и полет к тумбе
if direction == 0:
    sector = 'B'
    print("Sector B required")
    need_x = arrow_x
    need_y = arrow_y / 2
elif direction == 1:
    sector = 'D'
    print("Sector D required")
    need_x = arrow_x / 2
    need_y = arrow_y
elif direction == 2:
    sector = 'A'
    print("Sector A required")
    need_x = arrow_x
    need_y = arrow_y + (polygon_y - arrow_y) / 2
elif direction == 3:
    sector = 'C'
    print("Sector C required")
    need_x = arrow_x + (polygon_x - arrow_x) / 2
    need_y = arrow_y

# выбор высоты в зависимости от цвета и полет к тумбе
if arrow_color == 'black':
    need_z = 1.5
    indic = 'black'
elif arrow_color == 'red':
    need_z = 1.8
    indic = 'red'
elif arrow_color == 'blue':
    need_z = 1.5
    indic = 'blue'
elif arrow_color == 'yellow':
    need_z = 1.2
    indic = 'yellow'

print("Fly to find tumba")
navigate_wait(x=need_x, y=need_y, z=need_z, frame_id='aruco_map')
print("Got to point to find point")

rospy.sleep(3)
# посадка
preciseLanding(need_z, indic)

rospy.sleep(4)

set_effect(r=0, b=0, g=0)

print(str(num_order) + " delivered in Sector" + sector)

rospy.sleep(6)
# взлет
print("Takeoff")
navigate_wait(z=1, yaw=float('nan'), auto_arm=True, frame_id='body', speed=1.5, tolerance=0.3)
print("Took off")

print("Go to start point")
navigate_wait(x=0, y=0, z=FLY_HEIGHT, frame_id='aruco_map', speed=0.5)
print("Got to start point")
rospy.sleep(3)
print("Landing")

land()

# отписка от топиков и запись времени итогового
telem_sub.unregister()
thresh_sub.unregister()

f.close()
fly_time_sec = time.time() - start_time
fly_time_str = str(int(fly_time_sec // 60)) + ' min ' + str(int(fly_time_sec % 60)) + ' sec'
print(str(num_order) + " delivered in Sector" + sector + " for " + fly_time_str)

rospy.sleep(2)
