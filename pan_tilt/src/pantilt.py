
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

HARDCODED_PAN_LIMITS = (-0.5, 0.5) # these are just made up
HARDCODED_TILT_LIMITS = (-0.7, 0.1) # These are empirical but real

class PanTilt():
    def __init__(self):
        self.pan_sub = rospy.Subscriber("/pan_motor/joint_states", JointState, self.panstatecb, queue_size=1) 
        self.tilt_sub = rospy.Subscriber("/tilt_motor/joint_states", JointState, self.tiltstatecb, queue_size=1) 

        self.pan_pub = rospy.Publisher("/pan_motor/position_controller/command", Float64, queue_size=1)
        self.tilt_pub = rospy.Publisher("/tilt_motor/position_controller/command", Float64, queue_size=1)

        self.pan_theta = None
        self.tilt_theta = None

    def panstatecb(self, msg):
        self.pan_theta = msg.position[0]

    def tiltstatecb(self, msg):
        self.tilt_theta = msg.position[0]

    def tilt_go(self, tilt_target):
        if tilt_target < HARDCODED_TILT_LIMITS[0] or HARDCODED_TILT_LIMITS[1] < tilt_target:
            rospy.logerr(f"Invalid Target {tilt_target:1.5f} but limited to {HARDCODED_TILT_LIMITS}")
            return

        self.tilt_pub.publish(tilt_target)

    def pan_go(self, pan_target):
        if pan_target < HARDCODED_PAN_LIMITS[0] or HARDCODED_PAN_LIMITS[1] < pan_target:
            rospy.logerr(f"Invalid Target {pan_target:1.5f} but limited to {HARDCODED_pan_LIMITS}")
            return
        self.pan_pub.publish(pan_target)


if __name__ == "__main__":
    rospy.init_node("PanTilt_Client", anonymous=True)
    pt = PanTilt()
    print("Spinning pantilt")
    rospy.sleep(1.0)
    pt.tilt_go(0.0)
    rospy.sleep(2.0)
    pt.tilt_go(-0.3)
    rospy.sleep(2.0)
    pt.pan_go(0.2)
    rospy.sleep(2.0)
    pt.pan_go(-0.2)

    rospy.spin()

