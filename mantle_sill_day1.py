import rospy
import numpy as np
import time

import cv2 as cv
from cv_bridge import CvBridge

from clover import srv
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool
import math

bridge = CvBridge()

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

TIME_RATE = 0.1
TIME_UP = 3
TIME_DOWN = 3

FLY_HEIGHT = 1.4

debug = rospy.Publisher('debug', Image, queue_size=1)
detect = rospy.Publisher('/Detect', Image, queue_size=1)

relatives = {
        0: 'Down',
        1: 'Left',
        2: 'Up',
        3: 'Right'
    }

cx = 0
cy = 0

def navigate_wait(x=0, y=0, z=0, yaw=math.radians(90), speed=0.3, frame_id='', auto_arm=False, tolerance=0.15):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(TIME_RATE)

def find_arrow(data):
    print("finding Arrow")
    
    samples = np.loadtxt('generalsamples.data', np.float32)
    responses = np.loadtxt('generalresponses.data', np.float32)
    responses = responses.reshape((responses.size, 1))

    model = cv.ml.KNearest_create()
    model.train(samples, cv.ml.ROW_SAMPLE, responses)

    print("model made")

    ############################# testing part #########################

    im = bridge.imgmsg_to_cv2(data, 'bgr8')[110:150, 140:180]
    out = im.copy()
    gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    # thresh = cv.adaptiveThreshold(gray, 255, 1, 1, 11, 2)

    ret, thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY_INV)
    debug.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))
    print("published")

    contours, hierarchy = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[-2:]

    cnt = max(contours, key=cv.contourArea)

    num = 0

    [x, y, w, h] = cv.boundingRect(cnt)
    try:
        print("is contour")
        cv.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
        roi = thresh[y:y + h, x:x + w]
        roismall = cv.resize(roi, (10, 10))
        roismall = roismall.reshape((1, 100))
        roismall = np.float32(roismall)
        retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
        num = int(results[0][0]) # ?
        result = relatives[num]
        #cv.putText(out, result, (x + w // 2, y + h // 2), 0, 1, (0, 255, 0))
    except cv.Error as e:
        print('Invalid')

    cv.drawContours(out, [cnt], -1, (255, 105, 180), 3)

    detect.publish(bridge.cv2_to_imgmsg(out, 'bgr8'))

    print("Arrow is: " + relatives[num])

    return num

def getCenterOfContour_callback(data):
    global cx
    global cy
    im = CvBridge().imgmsg_to_cv2(data, 'bgr8')
    out = im.copy()
    hsv = cv.cvtColor(im, cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsv, (50, 55, 60), (85, 255, 229))

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

def preciseLanding():  
    global cx
    global cy
    z = 1.8
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, getCenterOfContour_callback, queue_size=1)
    rospy.sleep(1)

    x0, y0 = 320 // 2, 240 // 2

    while math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2) > 5:

        if math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2) < 50:
            z = 1.7
        if math.sqrt((x0 - cx) ** 2 + (y0 - cy) ** 2) < 20:
            z = 1.55

        telem = get_telemetry(frame_id='aruco_map')

        dx, dy = normalize(cx - x0, cy - y0)  # get final motion vector 
        dx /= 15  # limit the speed
        dy /= 15
        dy = -dy  # the y-axis of the frame is directed in the opposite direction of the y-axis of the marker map
        # z -= 0.03
        set_position(x=telem.x + dx, y=telem.y + dy, z=z, yaw=math.radians(90), frame_id='aruco_map')
        rospy.sleep(0.1)

    image_sub.unregister()
    land()
    rospy.sleep(1)
    arming(False)
    cx, cy = 0, 0

print("Started")
start_time = time.time()
print("start time:", start_time)
navigate(x=0, y=0, z=FLY_HEIGHT, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(TIME_UP)
print("Got off")

navigate_wait(x=0, y=0, z=FLY_HEIGHT, frame_id='aruco_map')
print("Got to Started point")

navigate_wait(x=1.2, y=1.2, z=FLY_HEIGHT, frame_id='aruco_map')
print("Got to start scan point")
rospy.sleep(2)

direction = find_arrow(rospy.wait_for_message('main_camera/image_raw', Image))

sector = ''

if direction == 0:
    print("Sector B required")
    sector = 'B'
    navigate_wait(x=1.2, y=0.6, z=1.7, frame_id='aruco_map')
elif direction == 1:
    print("Sector D required")
    sector = 'D'
    navigate_wait(x=0.6, y=1.2, z=1.7, frame_id='aruco_map')
elif direction == 2:
    print("Sector A required")
    sector = 'A'
    navigate_wait(x=1.2, y=2, z=1.7, frame_id='aruco_map')
elif direction == 3:
    print("Sector C required")
    sector = 'C'
    navigate_wait(x=2.2, y=1.2, z=1.7, frame_id='aruco_map')

preciseLanding()

rospy.sleep(5)

print("3 delivered in Sector" + sector)

arming(True)
print("Start returning")
navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(4)
print("Got off")

navigate_wait(x=0, y=0, z=FLY_HEIGHT, frame_id='aruco_map')
print("Got to Started point")

land()
fly_time_sec = time.time() - start_time
fly_time_str = str(int(fly_time_sec // 60)) + ' min ' + str(int(fly_time_sec % 60)) + ' sec'
print("3 delivered in Sector" + sector + " for " + fly_time_str)

rospy.sleep(2)
