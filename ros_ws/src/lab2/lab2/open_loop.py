import rclpy
from rclpy.node import Node

# control robots using geometry_msgs/msg/Twist
from geometry_msgs.msg import Twist

# this is a node
class OpenLoop(Node):

    def __init__(self):
        # Init this node with a name passed in as a argument
        super().__init__('open_loop')

        # recall that "create_publisher" 
        # takes 3 args: msg type, topic name, queue length
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # run at certain frequency
        timer_period = 1.0      # seconds. 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Turtle has two modes of operation: turn in place or move forward
        self.turning = False        # this flag keeps track of the turtle state

        # keep track of # of turns executed. stop moving after 2 turns
        self.turns_i = 0

        self.forward_msg = Twist()
        self.forward_msg.linear.x = 1.0    # move forward at 1 m/s 

        self.turn_msg = Twist()
        self.turn_msg.angular.z = 1.57     # rad/s. rotate 90 deg in place

    def timer_callback(self):

        if (self.turns_i >= 2):
            self.get_logger().info("Robot has arrived at (1, 1)!")
        else:
            if (self.turning):
                self.get_logger().info("Robot is Turning!")
                self.publisher_.publish(self.turn_msg)      # calling publisher
                self.turns_i +=1

            else:
                self.get_logger().info("Robot is going forward!")
                self.publisher_.publish(self.forward_msg)      # calling publisher
                
            # toggle mode of the robot
            self.turning = not self.turning
    
def main(args=None):
    rclpy.init(args=args)

    open_loop = OpenLoop()
    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(open_loop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
