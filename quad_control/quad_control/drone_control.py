import rclpy
from rclpy.node import Node

from quad_interfaces.msg import QuadCmd
import socket
import time
import json 

HOST = '192.168.18.101'  # Server IP (localhost for testing on local machine)
PORT = 65432  

class SendCommand(Node):

    def __init__(self):
        super().__init__('send_command')
        self.subscription = self.create_subscription(
            QuadCmd, '/quad_ctrl',
            self.ctrl_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.aux1 = 1000
        self.aux2 = 1000
        self.mode = 0
        self.arm = False
        self.pressed_4 = False
        self.pressed_5 = 0
        self.flag = 0
        self.drone_cmds = [1500, 1500, 900, 1500, self.aux1, self.aux2, self.flag]
    
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.bind((HOST, PORT))
            self.sock.listen()
            self.get_logger().info(f"Server listening on {HOST}:{PORT}...")
            self.conn, self.addr = self.sock.accept()
            self.get_logger().info(f"Connected by {self.addr}")
        except socket.error as e:
            self.get_logger().error(f"Failed to start TCP server: {e}")

    def ctrl_callback(self, msg):
        self.drone_cmds[0] = msg.roll
        self.drone_cmds[1] = msg.pitch 
        self.drone_cmds[2] = msg.throttle
        self.drone_cmds[3] = msg.yaw
        if msg.armed:
            self.drone_cmds[4] = 1800
        else:
            self.drone_cmds[4] = 1000
        
        
    
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
        print(self.drone_cmds)
        self.send_cmd_drone()
        


def main(args=None):
    rclpy.init(args=args)

    send_command = SendCommand()

    rclpy.spin(send_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    send_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()