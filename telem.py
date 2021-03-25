import rospy
from clover import srv
from std_msgs.msg import Float32MultiArray

rospy.init_node('telemxyz')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

telem_pub = rospy.Publisher("/telem", Float32MultiArray, queue_size=1)

while not rospy.is_shutdown():

    telem = get_telemetry(frame_id = 'aruco_map')

    telem_pub.publish(Float32MultiArray(data = (telem.x, telem.y, telem.z)))
    rospy.sleep(0.5)

print('done')