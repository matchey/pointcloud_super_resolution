#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import PCSR

class DencePublisher:
    def __init__(self):
        rospy.init_node("publish_dence_point_cloud", anonymous=True)
        self.subscriber = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.publisher = rospy.Publisher("/velodyne_points/dence", PointCloud2, queue_size=1)
        self.sr = PCSR.SuperResolution()

    def callback(self, pc2):
        self.sr.upSampling(pc2)
        self.publisher.publish(pc2)

def main():
    try:
        pub = DencePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__': main()

