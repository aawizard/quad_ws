import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from quad_interfaces.msg import QuadCmd
import socket
import time
import json 

HOST = '192.168.18.103'  # Server IP (localhost for testing on local machine)
PORT = 65432        # Port to listen on

class JoyNode(Node):

    def __init__(self):
        super().__init__('joystick')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)
        self.declare_parameter("use_drone", False)
        self.declare_parameter("use_sim", False)
        self.use_drone = self.get_parameter("use_drone").value
        self.use_sim = self.get_parameter("use_sim").value
        self.get_logger().info(f"Use drone: {self.use_drone}, Use sim: {self.use_sim}")
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(QuadCmd, '/quad_ctrl', 10)
        self.subscription  # prevent unused variable warning
        self.aux1 = 1000
        self.aux2 = 1000
        self.mode = 0
        self.arm = False
        self.pressed_4 = False
        self.pressed_5 = 0
        self.flag = 0
        self.drone_cmds = [1500, 1500, 900, 1500, self.aux1, self.aux2, self.flag]
        if self.use_drone:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.sock.bind((HOST, PORT))
                self.sock.listen()
                self.get_logger().info(f"Server listening on {HOST}:{PORT}...")
                self.conn, self.addr = self.sock.accept()
                self.get_logger().info(f"Connected by {self.addr}")
            except socket.error as e:
                self.get_logger().error(f"Failed to start TCP server: {e}")
        

        
    def listener_callback(self, msg):
        if self.flag<500 and self.use_drone:
            self.drone_cmds = [1500, 1500, 1000, 1500, self.aux1, self.aux2, self.flag]
            # self.send_drone_commands()
            print(self.flag)
        else:
            if msg.axes[0] <=0:
                yaw = int((msg.axes[0] * -450.0) + 1500.0)
            else :
                yaw = int((msg.axes[0] * -450.0) + 1500.0)
                
            if msg.axes[2] <=0:
                roll = int((msg.axes[2] * -450.0) + 1500.0)
            else :
                roll = int((msg.axes[2] * -450.0) + 1500.0)
                
            if msg.axes[4] < 1:
                throttle = int(((msg.axes[4] +1 )* -500.0) + 1900.0)
            else :
                throttle = 1000
                
            if msg.axes[3] >=0:
                pitch = int((msg.axes[3] * 450.0) + 1500.0)
            else :
                pitch = int((msg.axes[3] * 450.0) + 1500.0)
                
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
                    if self.arm:
                        self.aux1 = 1000  # Set aux1 for disarm
                        self.arm = False      # Disarm
                    else:
                        self.aux1 = 1800  # Set aux1 for arm
                        self.arm = True      # Arm
                        

            else:
                # Button is released
                if self.pressed_4:
                    self.get_logger().info("Button released.")
                    self.pressed_4 = False

            
            self.drone_cmds = [roll, pitch, throttle, yaw, self.aux1, self.aux2, self.flag]
            # self.send_drone_commands()
        
            # self.get_logger().info(f'yaw: {yaw}     roll: {roll}    pitch: {pitch}      throttle: {throttle} arm: {self.aux1}')
            
    def send_drone_commands_sim(self):
        cmd = QuadCmd()
        cmd.roll = self.drone_cmds[0]
        cmd.pitch = self.drone_cmds[1]
        cmd.throttle = self.drone_cmds[2]
        cmd.yaw = self.drone_cmds[3]
        cmd.armed = self.arm
        self.publisher_.publish(cmd)
    
    def send_cmd_drone(self):
        try:
            # Simulate control loop timing
            self.flag += 1  # Increment the flag to simulate changes over time
            self.drone_cmds[-1] = self.flag  # Update the flag in the command list

            # Send the command array as a JSON string
            data = json.dumps(self.drone_cmds) + '\n'
            self.conn.sendall(data.encode('utf-8'))
            
        except socket.error as e:
            self.get_logger().error(f"Failed to send data, error: {e}")
        
    def timer_callback(self):
        
        if self.use_drone:
            self.send_cmd_drone()
        if self.use_sim:
            self.send_drone_commands_sim()
        


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