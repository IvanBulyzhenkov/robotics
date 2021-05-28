import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

MARKER_ID_DETECTION = 9

class AutomaticParkingVision():
  def __init__(self):
    rospy.loginfo("__init__")
    self.sub_info_marker = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cbGetMarkerOdom, queue_size = 1)

  def cbGetMarkerOdom(self, markers_odom_msg):
    for marker_odom_msg in markers_odom_msg.markers:
      rospy.loginfo(marker_odom_msg.id)
    
  def main(self):
    rospy.spin()

if __name__ == '__main__':
  rospy.init_node('automatic_parking_vision')
  node = AutomaticParkingVision()
  node.main()
