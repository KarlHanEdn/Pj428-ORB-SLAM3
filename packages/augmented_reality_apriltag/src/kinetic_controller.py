import rospy
from duckietown_msgs.msg import WheelsCmdStamped


class KineticController:
    def __init__(self, cp, ci, cd):
        """
        cp, ci and cd are all need to be smaller than 0
        cp < 0 to minimize the error (go in the other direction of the error)
        ci < 0 for the same reason
        cd < 0 in order to minimize the derivative (counter the momentum of the vehicle)
        """
        self.pub = rospy.Publisher(f'/{HOST_NAME}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        
        def myhook():
            self.stop()
        rospy.on_shutdown(self.myhook)

        self.coeffs = (cp, ci, cd)
        self.error = 0
        self.int_error = 0
        self.adjustment = 0

    def stop(self):
        """
        stay still to reduce the momentum of the car to zero after carrying out some movement
        """
        self.drive(0, 0)

    def drive(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.pub.publish(msg)
    
    def update_error(self, error):
        self.int_error += error
        derivative = error - self.error
        self.error = error

        cp, ci, cd = self.coeffs

        self.adjustment = self.int_error * ci + self.error * cp + derivative * cd

    def get_adjustment():
        return self.adjustment

