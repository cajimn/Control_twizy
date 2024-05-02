import rospy
from std_msgs.msg import Header
from sd_msgs.msg import SDControl  # Replace with the actual package name

def move_forward():
    # Initialize the ROS node
    rospy.init_node('vehicle_control', anonymous=True)
    # Create a publisher object
    pub = rospy.Publisher('sd_control', SDControl, queue_size=1)
    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10 Hz

    # Create an instance of the SDControl message
    control_msg = SDControl()
    control_msg.header = Header()
    control_msg.header.stamp = rospy.Time.now()
    control_msg.torque = 50  # Adjust as needed for desired speed
    control_msg.steer = 0  # Go straight
    print("Controlling")
    
    while not rospy.is_shutdown():
        # Publish the control message
        pub.publish(control_msg)
        # Wait for the next time to publish
        rate.sleep()

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
