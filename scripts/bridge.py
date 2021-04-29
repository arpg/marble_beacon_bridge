#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool.msg
import estop_msgs

class BRIDGE:
    def __init__(self,robot_ns):
        # Create Subscriber to Base Station
        self.sub_deploy = rospy.Subscriber('/'+robot_ns+'/deploy',Bool(),self.deploy_cb)
        # Create Subscriber to Beacon Servos
        self.sub_servo = rospy.Subscriber('/'+robot_ns+'/servo_status',estop_msgs.ServoStatus(),self.status_cb)
        # Create Publisher to Beacon Drop
        self.pub_drop = rosppy.Publisher('/'+robot_ns+'/set_relay',estop_msgs.SetChannel.msg,queue_size=10)
        self.store_servo_info = [0,0,0,0,0,0,0,0]

    def status_cb(self,msg):
        self.store_servo_info = msg.position

    def deploy_cb(self,msg):
        drop = estop_msgs.SetChannel()
        drop.state = 1
        if self.store_servo_info[0] >= 750:
            drop.id = 0
            pub_drop(drop)
        elif self.store_servo_info[1] >= 750:
            drop.id = 1
            pub_drop(drop)
        elif self.store_servo_info[2] >= 750:
            drop.id = 2
            pub_drop(drop)
        else: 
            rospy.logwarn('No Beacons to Drop')
            


def main():
    rospy.init_node('beacon_bridge')
    robot_ns = rospy.get_param('~robot_ns','H01')
    create_bridge = BRIDGE(robot_ns)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
