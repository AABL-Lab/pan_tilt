
#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

HARDCODED_PAN_LIMITS = (-0.7, 0.7) # these are just made up
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
            rospy.logerr(f"Invalid Target {pan_target:1.5f} but limited to {HARDCODED_PAN_LIMITS}")
            return
        self.pan_pub.publish(pan_target)

    def increment_pantilt(self, pan_inc, tilt_inc):
        new_pan = self.pan_theta + pan_inc
        new_tilt = self.tilt_theta + tilt_inc
        print(f"Incrementing to {new_pan:1.2f} {new_tilt:1.2f}")
        self.pan_go(new_pan)
        self.tilt_go(new_tilt)



if __name__ == "__main__":
    rospy.init_node("PanTilt_Client", anonymous=True)
    pt = PanTilt()
    tilt0 = 0.0
    tilt1 = -0.3
    pan0 = 0.2
    pan1 = -0.2
    print("Spinning pantilt")
    rospy.sleep(1.0)
    print("Tilt to ", tilt0)
    pt.tilt_go(tilt0)
    rospy.sleep(2.0)

    print("Tilt to ", tilt1)
    pt.tilt_go(tilt1)
    rospy.sleep(2.0)

    print("Pan to ", pan0)
    pt.pan_go(pan0)
    rospy.sleep(2.0)

    print("Pan to ", pan1)
    pt.pan_go(pan1)

    rospy.spin()

