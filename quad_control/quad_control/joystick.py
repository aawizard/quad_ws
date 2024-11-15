import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from quad_interfaces.msg import QuadCmd

class JoyNode(Node):

    def __init__(self):
        super().__init__('joystick')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(QuadCmd, '/quad_ctrl', 10)
        self.subscription  # prevent unused variable warning
        self.cmd = QuadCmd()
        self.cmd.yaw = 1500
        self.cmd.pitch = 1500
        self.cmd.roll = 1500
        self.cmd.yaw = 1000
        self.cmd.armed = False
        self.pressed_4 = False

        
    def listener_callback(self, msg):
        if msg.axes[0] <=0:
            self.cmd.yaw = int((msg.axes[0] * -450.0) + 1500.0)
        else :
            self.cmd.yaw = int((msg.axes[0] * -450.0) + 1500.0)
            
        if msg.axes[2] <=0:
            self.cmd.roll = int((msg.axes[2] * -450.0) + 1500.0)
        else :
            self.cmd.roll = int((msg.axes[2] * -450.0) + 1500.0)
            
        if msg.axes[4] < 1:
            self.cmd.throttle = int(((msg.axes[4] +1 )* -500.0) + 1900.0)
        else :
            self.cmd.throttle = 1000
            
        if msg.axes[3] >=0:
            self.cmd.pitch = int((msg.axes[3] * 450.0) + 1500.0)
        else :
            self.cmd.pitch = int((msg.axes[3] * 450.0) + 1500.0)
            
        # if msg.buttons[4] == 1 and not self.pressed_4:
        #     self.pressed_4 = 1                                                                           
            
        # Assume this is part of your subscriber callback function

        # Check button state (arm button)
        if msg.buttons[9] == 1:
            # Button is pressed
            if not self.pressed_4:
                # This is the first press
                self.pressed_4 = True
                self.get_logger().info("Button pressed, toggling arm state.")

                # Toggle the arm state
                if self.cmd.armed:
                    self.cmd.armed = False  # Set aux1 for disarm
                         # Disarm
                else:
                    self.cmd.armed = True  # Set aux1 for arm
                    # self.arm = True      # Arm
                    

        else:
            # Button is released
            if self.pressed_4:
                self.get_logger().info("Button released.")
                self.pressed_4 = False  
    
        
    def timer_callback(self):
        self.publisher_.publish(self.cmd)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = JoyNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()