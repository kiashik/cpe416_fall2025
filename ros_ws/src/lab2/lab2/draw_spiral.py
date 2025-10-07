import rclpy
from rclpy.node import Node

# control robots using geometry_msgs/msg/Twist
from geometry_msgs.msg import Twist


# this is a node
class DrawSpiral(Node):

    def __init__(self):
        # Init this node with a name passed in as a argument
        super().__init__('draw_spiral')

        # recall that "create_publisher" 
        # takes 3 args: msg type, topic name, queue length
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # run at certain frequency
        timer_period = 0.1      # seconds. 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.t = 0.0
        self.linear_vel = 0.05
        self.angular_vel = 0.6
        self.linear_acc = 0.002
        self.angular_decay = 0.0005
        
        self.forward_msg = Twist() 
        self.forward_msg.linear.x = self.linear_vel     
        self.forward_msg.angular.z = self.angular_vel    


#        self.turn_msg = Twist()
 #       self.turn_msg.angular.z = 1.0    # rad/s. rotate ~14 deg in place

    def timer_callback(self):
        self.forward_msg.linear.x += self.linear_acc
        
        self.forward_msg.angular.z = max(0.1, self.forward_msg.angular.z - self.angular_decay)
        
        self.publisher_.publish(self.forward_msg)
    
    
def main(args=None):
    rclpy.init(args=args)

    draw_spiral = DrawSpiral()
    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(draw_spiral)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
