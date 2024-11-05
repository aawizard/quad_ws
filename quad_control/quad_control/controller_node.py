import rclpy
from rclpy.node import Node
from quad_interfaces.msg import QuadCmd
from controllers.pid_altitude import PID_alttitude
from geometry_msgs.msg import PoseStamped
import socket
import time
import json 


HOST = '192.168.18.103'  # Server IP (localhost for testing on local machine)
PORT = 65433        # Port to listen on

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.declare_parameter("use_drone", False)
        self.declare_parameter("use_sim", False)
        self.use_drone = self.get_parameter("use_drone").value
        self.use_sim = self.get_parameter("use_sim").value
        
        self.publisher_ = self.create_publisher(QuadCmd, '/quad_ctrl', 10)
        self.subscription = self.create_subscription(PoseStamped, '/quad_pose', self.listener_callback, 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.pid_altitude = PID_alttitude(kp=0.7, ki=0.0, kd=0.1, dt=timer_period)
        self.desired_altitude = 1.0
        self.curr_altitude = 0.0
        self.quad_cmd = QuadCmd()
        self.aux1 = 1000
        self.aux2 = 1000
        self.mode = 0
        self.flag = 0
        self.arm = False
        self.drone_cmds = [1500, 1500, 1000, 1500, self.aux1, self.aux2, self.flag]
        
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
        
    def set_desired_altitude(self, altitude):
        self.desired_altitude = altitude
        
    def listener_callback(self, msg):
        self.curr_altitude = msg.pose.position.z
                
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
        error = self.desired_altitude - self.curr_altitude
        thrust = self.pid_altitude.step(error)
        if self.use_sim:
            self.quad_cmd.throttle = thrust
            self.quad_cmd.armed = True
            self.publisher_.publish(self.quad_cmd)
        
        if self.use_drone:
            
            if self.flag > 500 and self.flag<700:
                print("arming")
                self.aux1 = 1800
                self.drone_cmds = [1500, 1500, 1000, 1500, 1800, self.aux2, self.flag]
                self.send_cmd_drone()
            if self.flag > 700:
                self.aux1 = 1800
                self.drone_cmds = [1500, 1500, thrust, 1500, self.aux1, self.aux2, self.flag]
                self.send_cmd_drone()
            
            else:
              self.flag += 1
              print(self.flag)
              self.send_cmd_drone()

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()