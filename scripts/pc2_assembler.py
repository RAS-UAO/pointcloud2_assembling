#!/usr/bin/env python3
import roslib
roslib.load_manifest('laser_assembler')
import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

"""
<include file="$(find vonns0_reconstruction3d)/launch/pc2_lidar.launch"/>
<node pkg= "vonns0_reconstruction3d" type= "pc2_assembler.py" name= "pointcloud2_assembler"/>
"""

class Pointcloud2AssemblerNode:
    def __init__(self):
        rospy.wait_for_service("/assemble_scans2")

        self.rate = rospy.Rate(1)

        self.assemble_scans2 = rospy.ServiceProxy('/assemble_scans2', AssembleScans2)

        self.rviz_publisher = rospy.Publisher("/pointcloud2", PointCloud2, queue_size=10)
    
    def assemble_pointcloud2(self):
        try:
            pointcloud2_response = self.assemble_scans2(rospy.Time(0,0), rospy.get_rostime())
            self.rviz_publisher.publish(pointcloud2_response.cloud)
            
            self.rate.sleep()
        except rospy.ServiceException:
            print("Point clouds failed to assemble")

if __name__ == "__main__":
    rospy.init_node("pointcloud2_assembler_client")
    
    node = Pointcloud2AssemblerNode()

    while not rospy.is_shutdown():
        node.assemble_pointcloud2()
