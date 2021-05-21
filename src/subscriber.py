import math

import rospy
from rospy.core import is_shutdown

from std_msgs.msg import Float32

x = 0
y = 0
phi = 0

def xCallback(data):
  x = data.data
  rospy.loginfo("x = %f", data.data)

def yCallback(data):
  y = data.data
  rospy.loginfo("y = %f", data.data)

def phiCallback(data):
  phi = data.data
  rospy.loginfo("phi = %f", data.data)

if __name__ == "__main__":
  k_v = 0.5
  k_w = 0.5

  x_ref = 80
  y_ref = -100
  phi_ref = math.atan(x_ref / y_ref)

  v_cur = 0
  w_cur = 0  

  v = Float32()
  w = Float32()

  rospy.init_node("listener")
  
  x_sub = rospy.Subscriber("x_pub", Float32, xCallback)
  y_sub = rospy.Subscriber("y_pub", Float32, yCallback)
  phi_sub = rospy.Subscriber("phi_pub", Float32, phiCallback)

  v_pub = rospy.Publisher("v_pub", Float32, queue_size=10)
  w_pub = rospy.Publisher("w_pub", Float32, queue_size=10)

  loop_rate = rospy.Rate(10)

  tolerance = 0.000001

  while (not rospy.is_shutdown()):
    if (abs(x - x_ref) < tolerance and abs(y - y_ref) < tolerance):
      print("Goal reached")
      break

    v_cur = -k_v * ((x - x_ref) * math.sin(phi) + (y - y_ref) * math.cos(phi))
    if (v_cur > 0.5):
      v_cur = 0.5
    if (v_cur < -0.5):
      v_cur = -0.5

    v.data = v_cur

    phi_ref = math.atan((x - x_ref) / (y - y_ref))
    w_cur = -k_w * (phi - phi_ref)
    if (w_cur > 0.01):
      w_cur = 0.01
    if (w_cur < -0.01):
      w_cur = -0.01

    w.data = w_cur

    v_pub.publish(v)
    w_pub.publish(w)

    loop_rate.sleep()